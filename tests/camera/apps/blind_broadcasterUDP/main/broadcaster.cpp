#include "details.h"

#include <memory>
#include <atomic>
#include <array>
#include <chrono>
#include <algorithm>
#include <cerrno>
#include "esp_timer.h"

using asio::ip::tcp;
using asio::ip::udp;

/*
Protocol (no commands):
  - A client opens a TCP connection to SERVER_PORT. That connection only signals "I am here":
    the server never reads commands from it, it just notices when it closes.
  - While the TCP connection is open, every camera frame is sent over UDP to the client's
    address, port CLIENT_UDP_PORT.
  - When the TCP connection closes, streaming stops and the server waits for the next client.

Each JPEG is split into packets of at most UDP_PAYLOAD bytes, each prefixed with PacketHeader.
A lost packet only loses that one frame; nothing waits for retransmissions.
*/
static constexpr unsigned short CLIENT_UDP_PORT = 3334;
static constexpr size_t UDP_PAYLOAD = 1400;              // + 12-byte header + 28 IP/UDP = 1440 < 1500 MTU

#pragma pack(push, 1)
struct PacketHeader {               // little-endian on the wire (ESP32 native order)
    uint32_t frame_id;              // increments per frame, restarts at 0 for every new client
    uint16_t packet_index;          // 0 .. packet_count-1
    uint16_t packet_count;          // packets in this frame
    uint32_t frame_size;            // total JPEG bytes in this frame
};
#pragma pack(pop)
static_assert(sizeof(PacketHeader) == 12, "header must be 12 bytes");

// ============================ frame ownership ===============================

struct FbReturn {
    void operator()(camera_fb_t* fb) const { esp_camera_fb_return(fb); }
};
using FramePtr = std::unique_ptr<camera_fb_t, FbReturn>;   // destroyed = returned to the camera

static std::atomic<bool> capture_enabled{false};   // the only state shared across threads
static TaskHandle_t capture_handle = nullptr;

static void enable_capture(bool on)
{
    capture_enabled = on;
    if (on) xTaskNotifyGive(capture_handle);
}

// ============================ server ========================================

/*
One client at a time. Connecting starts the stream, disconnecting stops it.
*/
class Server{
    tcp::acceptor accept_incoming;
    tcp::socket socket;
    udp::socket udp_socket;
    udp::endpoint client_udp;               // where frames are sent
    asio::steady_timer retry_timer;         // waits briefly when the Wi-Fi TX queue is full

    char discard[64];                       // anything the client sends is read and ignored
    bool listining = false;

    FramePtr pending;                       // newest frame waiting to be sent
    FramePtr in_flight;                     // frame whose packets are being sent
    PacketHeader header{};                  // header of the packet currently being sent
    uint32_t next_frame_id = 0;

    uint32_t stat_frames = 0, stat_replaced = 0, stat_retries = 0;
    int64_t  stat_t0 = 0, stat_send_us = 0, frame_t0 = 0;

public:
    Server(asio::io_context& context, unsigned short port)
        : accept_incoming{context}, socket{context}, udp_socket{context}, retry_timer{context}
    {
        asio::post(context, [this, port](){
            std::error_code ec;
            accept_incoming.open(tcp::v4(), ec);
            if(!ec)
                accept_incoming.set_option(tcp::acceptor::reuse_address(true), ec);
            if(!ec)
                accept_incoming.bind(tcp::endpoint(tcp::v4(), port), ec);
            if(!ec)
                accept_incoming.listen(1, ec);
            if(!ec)
                udp_socket.open(udp::v4(), ec);

            if(!ec){
                ESP_LOGW(TAG, "accepting incoming connection");
                async_accept_client();
            } else {
                ESP_LOGE(TAG, "server setup failed: %s", ec.message().c_str());
            }
        });
    }

    // Called on the io_context thread (posted by the capture task).
    void on_frame(FramePtr f)
    {
        if (!socket.is_open()) return;      // no client: f is returned to the camera here
        if (pending) stat_replaced++;
        pending = std::move(f);             // an older pending frame (if any) is returned automatically
        try_send();
    }

private:
    void async_accept_client(){
        if(listining)
            return;
        listining = true;
        accept_incoming.async_accept([this](std::error_code ec, tcp::socket new_socket) {
            listining = false;
            if(ec){
                async_accept_client();
                return;
            }
            ESP_LOGW(TAG, "client connected");
            socket = std::move(new_socket);

            std::error_code ignored;
            socket.set_option(tcp::no_delay(true), ignored);

            // frames go to the same host that opened the TCP connection
            auto peer = socket.remote_endpoint(ignored);
            client_udp = udp::endpoint(peer.address(), CLIENT_UDP_PORT);
            ESP_LOGI(TAG, "streaming to %s:%u", peer.address().to_string().c_str(), CLIENT_UDP_PORT);

            next_frame_id = 0;
            stat_t0 = esp_timer_get_time();
            watch_connection();
            enable_capture(true);           // start blasting frames
        });
    }

    // Keeps one read pending on the TCP socket. Its only purpose is to notice
    // when the client goes away (read fails or returns end-of-file).
    void watch_connection(){
        socket.async_read_some(asio::buffer(discard),
        [this](std::error_code ec, std::size_t) {
            if (ec) { drop_client_and_listen_to_incoming(); return; }
            watch_connection();             // client sent something: ignore it, keep watching
        });
    }

    // ---------------------------- UDP frame sending -------------------------

    void try_send()
    {
        if (in_flight || !pending) return;  // one frame at a time; nothing to send
        in_flight = std::move(pending);
        frame_t0 = esp_timer_get_time();

        header.frame_id     = next_frame_id++;
        header.frame_size   = in_flight->len;
        header.packet_count = (in_flight->len + UDP_PAYLOAD - 1) / UDP_PAYLOAD;
        header.packet_index = 0;
        send_packet();
    }

    void send_packet()
    {
        if (!socket.is_open()) { finish_frame(); return; }     // client gone: stop sending this frame

        size_t offset = size_t(header.packet_index) * UDP_PAYLOAD;
        size_t n = std::min(UDP_PAYLOAD, in_flight->len - offset);
        std::array<asio::const_buffer, 2> bufs{
            asio::buffer(&header, sizeof(header)),
            asio::buffer(in_flight->buf + offset, n)};

        udp_socket.async_send_to(bufs, client_udp, [this](std::error_code ec, std::size_t) {
            // Check the raw errno: on the ESP32 a full TX queue shows up as ENOMEM
            // ("Not enough space"), and comparing via std::errc did not match there.
            if (ec.value() == ENOMEM || ec.value() == ENOBUFS) {
                stat_retries++;
                retry_timer.expires_after(std::chrono::milliseconds(2));
                retry_timer.async_wait([this](std::error_code) { send_packet(); });
                return;
            }
            if (ec) {
                ESP_LOGW(TAG, "udp send failed: %s (frame dropped)", ec.message().c_str());
                finish_frame();
                return;
            }
            if (++header.packet_index < header.packet_count) send_packet();
            else finish_frame();
        });
    }

    void finish_frame()
    {
        if (in_flight) {
            stat_send_us += esp_timer_get_time() - frame_t0;
            if (++stat_frames == 30) {
                int64_t now = esp_timer_get_time();
                ESP_LOGI(TAG, "sent %.1f fps | send %.1f ms/frame | replaced %u | queue-full retries %u",
                         30e6 / (now - stat_t0), stat_send_us / 30e3, (unsigned)stat_replaced, (unsigned)stat_retries);
                stat_t0 = now; stat_frames = 0; stat_send_us = 0; stat_replaced = 0; stat_retries = 0;
            }
        }
        in_flight.reset();                  // buffer goes back to the camera
        try_send();                         // a newer frame may already be waiting
    }

    // ---------------------------- disconnect --------------------------------

    void drop_client_and_listen_to_incoming() {
        enable_capture(false);
        pending.reset();
        if (socket.is_open()) {
            std::error_code ignored;
            socket.shutdown(tcp::socket::shutdown_both, ignored);
            socket.close(ignored);
            ESP_LOGW(TAG, "client disconnected, stream stopped");
        }
        async_accept_client();
    }
};

// ============================ capture task ==================================

struct CaptureCtx { asio::io_context* io; Server* server; };

static void capture_task(void* arg)
{
    auto* ctx = static_cast<CaptureCtx*>(arg);
    while (true) {
        if (!capture_enabled.load()) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // sleep until enabled
            continue;
        }
        FramePtr f(esp_camera_fb_get());                // blocks until a frame AND a free buffer
        if (!f) continue;
        asio::post(*ctx->io, [s = ctx->server, f = std::move(f)]() mutable {
            s->on_frame(std::move(f));                  // runs on the io_context thread
        });
    }
}

// ============================ main ==========================================

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (ESP_OK != init_camera()) {
        return;
    }
    if (sensor_t *sensor = esp_camera_sensor_get()) {
        sensor->set_aec2(sensor, 0);                        // night mode off: exposure <= one frame
        sensor->set_exposure_ctrl(sensor, 1);
        sensor->set_gain_ctrl(sensor, 1);
        sensor->set_gainceiling(sensor, GAINCEILING_16X);   // dark scenes: more gain, not longer frames
        sensor->set_pll(sensor, 0, 224, 4, 0, 2, 2, 1, 4);  // 16 MHz XCLK -> ~45 MHz SYSCLK
    }

    wifi_init_softap();

    asio::io_context io_context;
    Server s(io_context, SERVER_PORT);

    static CaptureCtx ctx{&io_context, &s};
    xTaskCreatePinnedToCore(capture_task, "capture", 4096, &ctx, 5, &capture_handle, 1);
    vTaskPrioritySet(nullptr, 6);           // Asio thread above the capture task
    io_context.run();
}