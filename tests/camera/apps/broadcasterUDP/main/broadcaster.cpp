#include "details.h"

#include <memory>
#include <atomic>
#include <array>
#include <string_view>
#include <chrono>
#include <algorithm>
#include "esp_timer.h"

using asio::ip::tcp;
using asio::ip::udp;

/*
Commands arrive over TCP (unchanged): 4-byte little-endian length + "stream" | "stop" | "snapshot".
Frames go out over UDP to the TCP client's address, port CLIENT_UDP_PORT.

Each JPEG is split into packets of at most UDP_PAYLOAD bytes, each prefixed with PacketHeader.
A lost packet only loses that one frame; nothing waits for retransmissions.
*/
static constexpr unsigned short CLIENT_UDP_PORT = 3334;
static constexpr size_t UDP_PAYLOAD = 1400;              // + 12-byte header + 28 IP/UDP = 1440 < 1500 MTU

#pragma pack(push, 1)
struct PacketHeader {               // little-endian on the wire (ESP32 native order)
    uint32_t frame_id;              // increments per frame
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
This is a server that can only connect to one client (we shall see if we can manage more..)
*/
class Server{
    tcp::acceptor accept_incoming;
    tcp::socket socket;
    udp::socket udp_socket;
    udp::endpoint client_udp;               // where frames are sent
    asio::steady_timer retry_timer;         // waits briefly when the Wi-Fi TX queue is full

    static constexpr uint32_t max_length = 1024;
    char data_incoming[max_length];
    char data_outgoing[max_length];
    bool listining = false;
    bool writing = false;                   // TCP write (error message) in progress

    enum mode{
        STREAM,
        SNAPSHOT
    } working_mode = SNAPSHOT;
    bool snapshot_requested = false;

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
        if (!socket.is_open()) return;                                       // no client: f returned here
        if (working_mode == mode::SNAPSHOT && !snapshot_requested) return;   // not requested: returned
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
            if(!ec){
                ESP_LOGW(TAG, "client connected");
                socket = std::move(new_socket);

                std::error_code ignored;
                socket.set_option(tcp::no_delay(true), ignored);

                // frames go to the same host that opened the TCP connection
                auto peer = socket.remote_endpoint(ignored);
                client_udp = udp::endpoint(peer.address(), CLIENT_UDP_PORT);
                ESP_LOGI(TAG, "frames will be sent to %s:%u",
                         peer.address().to_string().c_str(), CLIENT_UDP_PORT);

                read_incoming_size();
            } else {
                async_accept_client();
            }
        });
    }

    void read_incoming_size(){
        asio::async_read(socket, asio::buffer(data_incoming, sizeof(uint32_t)),
        [this](std::error_code ec, std::size_t) {
            if (ec) { drop_client_and_listen_to_incoming(); return; }

            uint32_t incoming_message_size = 0;
            std::memcpy(&incoming_message_size, &data_incoming[0], sizeof(uint32_t));
            if(incoming_message_size > max_length){
                static constexpr std::string_view message = "the incoming message is too big";
                warn_error(message);
            } else {
                read_remaining_message(incoming_message_size);
            }
        });
    }

    void read_remaining_message(size_t incoming_message){
        asio::async_read(socket, asio::buffer(data_incoming, incoming_message),
        [this](std::error_code ec, std::size_t length) {
            if (ec) { drop_client_and_listen_to_incoming(); return; }

            std::string_view cmd(data_incoming, length);
            if ("stream" == cmd) {
                working_mode = mode::STREAM;
                enable_capture(true);
            } else if ("snapshot" == cmd) {
                working_mode = mode::SNAPSHOT;
                snapshot_requested = true;
                enable_capture(true);
            } else if ("stop" == cmd) {
                working_mode = mode::SNAPSHOT;
                snapshot_requested = false;
                enable_capture(false);
                pending.reset();
            } else {
                static constexpr std::string_view message = "supported requests are \"stop\",\"stream\",\"snapshot\"";
                warn_error(message);
                return;
            }
            read_incoming_size();           // always keep a command read pending
        });
    }

    // ---------------------------- UDP frame sending -------------------------

    void try_send()
    {
        if (in_flight || !pending) return;  // one frame at a time; nothing to send
        in_flight = std::move(pending);
        frame_t0 = esp_timer_get_time();
        if (working_mode == mode::SNAPSHOT) { snapshot_requested = false; enable_capture(false); }

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
            if (ec == std::errc::not_enough_memory || ec == std::errc::no_buffer_space) {
                // lwIP / Wi-Fi TX queue is full: wait a moment, then resend the same packet
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

    // ---------------------------- TCP error message -------------------------

    void warn_error(std::string_view view)
    {
        enable_capture(false);
        pending.reset();
        if (writing) { drop_client_and_listen_to_incoming(); return; }

        uint32_t message_size = view.size();
        std::memcpy(&data_outgoing[0], &message_size, sizeof(uint32_t));
        std::memcpy(&data_outgoing[sizeof(uint32_t)], view.data(), view.size());
        writing = true;
        asio::async_write(socket, asio::buffer(data_outgoing, sizeof(uint32_t) + view.size()),
        [this](std::error_code, std::size_t) {
            writing = false;
            drop_client_and_listen_to_incoming();
        });
    }

    void drop_client_and_listen_to_incoming() {
        enable_capture(false);
        pending.reset();
        snapshot_requested = false;
        working_mode = mode::SNAPSHOT;
        if (socket.is_open()) {
            std::error_code ignored;
            socket.shutdown(tcp::socket::shutdown_both, ignored);
            socket.close(ignored);
            ESP_LOGW(TAG, "killed client");
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
        //sensor->set_pll(sensor, 0, 180, 2, 0, 2, 2, 1, 8);
        sensor->set_pll(sensor, 0, 224, 4, 0, 2, 2, 1, 4);

    }

    wifi_init_softap();

    asio::io_context io_context;
    Server s(io_context, SERVER_PORT);

    static CaptureCtx ctx{&io_context, &s};
    xTaskCreatePinnedToCore(capture_task, "capture", 4096, &ctx, 5, &capture_handle, 1);
    vTaskPrioritySet(nullptr, 6);
    io_context.run();
}