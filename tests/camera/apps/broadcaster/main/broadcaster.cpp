#include "details.h"

using asio::ip::tcp;

#include <memory>
#include <atomic>

struct FbReturn {
    void operator()(camera_fb_t* fb) const { esp_camera_fb_return(fb); }
};
using FramePtr = std::unique_ptr<camera_fb_t, FbReturn>;

static std::atomic<bool> capture_enabled{false};   // the only state shared across threads
static TaskHandle_t capture_handle = nullptr;

static void enable_capture(bool on)
{
    capture_enabled = on;
    if (on) xTaskNotifyGive(capture_handle);
}

/*
This is a server that can only connect to one client (we shall see if we can manage more..)
*/
class Server{
    tcp::acceptor accept_incoming;
    tcp::socket socket;
    static constexpr uint32_t max_length = 1024;
    char data_incoming[max_length];
    char data_outgoing[max_length];
    bool termination_flag = false; // there is only one context, so there is no need for this to be shared
    bool listining = false;
    bool writing = false;
    bool snapshot_requested = false;


    int64_t t_send_start = 0, sum_grab = 0, sum_send = 0;
    int frames = 0;
    size_t sum_bytes = 0;

    enum mode{
        STREAM,
        SNAPSHOT
    } working_mode = SNAPSHOT;

    FramePtr pending; 
    FramePtr in_flight;
    uint32_t frame_len;

public:
    Server(asio::io_context& context, unsigned short port) : accept_incoming{context}, socket{context}{

        context.post([&,port=port](){
            // if there are no exceptions then we can't execute failable code inside the constructor
            std::error_code ec;
            accept_incoming.open(tcp::v4(),ec);
            if(!ec) 
                accept_incoming.set_option(tcp::acceptor::reuse_address(true), ec);
            if(!ec) 
                accept_incoming.bind(tcp::endpoint(tcp::v4(), port), ec);
            if(!ec) 
                accept_incoming.listen(1, ec);

            if(!ec){
                ESP_LOGW(TAG,"accepting incoming connection");
                async_accept_client();
            }
                
        });
    }


    void async_accept_client(){
        if(listining)
            return;
        termination_flag = false;
        listining = true;
        accept_incoming.async_accept([&](std::error_code ec, tcp::socket new_socket) {
            if(!ec){
                listining = false;
                ESP_LOGW(TAG,"client connected");
                socket = std::move(new_socket);
                std::error_code ignored;
                socket.set_option(tcp::no_delay(true), ignored);
                if (ignored) ESP_LOGW(TAG, "no_delay failed: %s", ignored.message().c_str());
                read_incoming_size();
            } else {
                listining = false;
                async_accept_client();
            }
        });
    }


    void read_incoming_size(){
        asio::async_read(socket, asio::buffer(data_incoming, sizeof(uint32_t)),[&](std::error_code ec, std::size_t length) {
            if (!ec) { 
                uint32_t incoming_message_size = 0;
                std::memcpy(&incoming_message_size,&data_incoming[0],sizeof(uint32_t));
                if(incoming_message_size > max_length){ 
                    // the incoming message size is too big
                    // drop it, scream at the client and kill the connection
                    static constexpr std::string_view message = "the incoming message is too big";
                    //we can't accept right away because first we need to send message to the outside world
                    //only then do we kill the client
                    warn_error(message);
                } else {
                    read_remaining_message(incoming_message_size);
                }
            } else {
                drop_client_and_listen_to_incoming();
            }
        });
    }


    void read_remaining_message(size_t incoming_message){
        asio::async_read(socket, asio::buffer(data_incoming, incoming_message),[&,incoming_message](std::error_code ec, std::size_t length) {
            if (!ec) { 
                std::string_view cmd(data_incoming, incoming_message);
                if ("stream" == cmd)        { working_mode = mode::STREAM;   enable_capture(true); }
                else if ("snapshot" == cmd) { working_mode = mode::SNAPSHOT; enable_capture(true); }
                else if ("stop" == cmd)     { enable_capture(false); pending.reset(); }
                else {
                    //unknown so we report back to client and kill connection
                    static constexpr std::string_view message = "supported requests are \"stop\",\"stream\",\"snapshot\"";
                    //we can't accept right away because first we need to send message to the outside world
                    //only then do we kill the client
                    warn_error(message);
                    return;
                } 
                read_incoming_size();

            } else {
                drop_client_and_listen_to_incoming();
            }
        });
    }

    void on_frame(FramePtr f) {
       if (!socket.is_open()) return;
       if (working_mode == mode::SNAPSHOT && !snapshot_requested) return;   // unrequested: returned
       pending = std::move(f);
       try_send();
    }


    void try_send()
    {
        if (writing || in_flight || !pending) 
            return;
        in_flight = std::move(pending);
        if (working_mode == mode::SNAPSHOT) { snapshot_requested = false; enable_capture(false); }
        frame_len = in_flight->len;
        std::array<asio::const_buffer, 2> bufs{
            asio::buffer(&frame_len, sizeof(frame_len)),
            asio::buffer(in_flight->buf, in_flight->len)};

        asio::async_write(socket, bufs, [this](std::error_code ec, std::size_t) {
            in_flight.reset();              // sent (or failed): buffer goes back to the camera
            if (ec) { drop_client_and_listen_to_incoming(); return; }
            try_send();                     // a newer frame may already be waiting
        });
    }


    /*
    void grab_frame_and_write(){
        if (writing) return;

        int64_t t0 = esp_timer_get_time();
        camera_fb_t *pic = esp_camera_fb_get();
        int64_t t1 = esp_timer_get_time();
        sum_grab += t1 - t0;
        t_send_start = t1;

        if(pic){
            uint32_t frame_len = pic->len;
            std::memcpy(&data_outgoing[0],&frame_len,sizeof(uint32_t));
            std::array<asio::const_buffer, 2> bufs{
                asio::buffer(&data_outgoing, sizeof(uint32_t)),
                asio::buffer(pic->buf, pic->len)};
            writing = true;
            
            asio::async_write(socket, bufs,
            [&,pic](std::error_code ec, std::size_t length) {
                // we always return back the pointer we obtained, only then do we deal with errors 
                writing = false; 
                esp_camera_fb_return(pic);

                sum_send += esp_timer_get_time() - t_send_start;
                sum_bytes += length;
                if (++frames == 10) {
                    ESP_LOGI(TAG, "avg grab %.1f ms | avg send %.1f ms | avg %u bytes",
                            sum_grab / 10e3, sum_send / 10e3, (unsigned)(sum_bytes / 10));
                    frames = 0; sum_grab = sum_send = 0; sum_bytes = 0;
                }

                if (!ec) {
                    if(termination_flag){
                        termination_flag = false;
                        // we just take a rest
                        return;
                    }
                    if (working_mode==mode::STREAM){
                        grab_frame_and_write();
                    }
                } else {
                    drop_client_and_listen_to_incoming();
                }
            });
        } else {
            //kill socket
            drop_client_and_listen_to_incoming();
        }     
    }*/


    void warn_error(const std::string_view& view) {
       enable_capture(false);
       pending.reset();
       if (in_flight) { drop_client_and_listen_to_incoming(); return; }

        uint32_t message_size = view.size();
        std::memcpy(&data_outgoing[0],&message_size,sizeof(uint32_t));
        std::memcpy((&data_outgoing[0])+sizeof(uint32_t),view.data(),view.size()); 
        writing = true;
        asio::async_write(socket, asio::buffer(data_outgoing, view.size()+sizeof(uint32_t)),
        [this](std::error_code ec, std::size_t length) {
            writing = false;
            drop_client_and_listen_to_incoming();
        });
    }

    void drop_client_and_listen_to_incoming() {
        std::error_code ignored;
        socket.shutdown(tcp::socket::shutdown_both, ignored);
        socket.close(ignored);
        enable_capture(false);
        pending.reset();
        snapshot_requested = false;
        async_accept_client();
        ESP_LOGW(TAG,"killed client");
    }


};

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


extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();

    #if ESP_CAMERA_SUPPORTED
        if(ESP_OK != init_camera()) {
            return;
        }

        sensor_t *cam = esp_camera_sensor_get();
        if (cam) {
            cam->set_aec2(cam, 0);                       // night mode off: exposure never longer than one frame
            cam->set_exposure_ctrl(cam, 0);              // auto exposure, within that limit
            cam->set_gain_ctrl(cam, 1);                  // auto gain
            cam->set_gainceiling(cam, GAINCEILING_32X);  // dark scenes: more gain instead of longer frames
            cam->set_pll(sensor, 0, 180, 2, 0, 2, 2, 1, 8);
        }

    #if defined(CONFIG_CAMERA_AF_SUPPORT) && CONFIG_CAMERA_AF_SUPPORT
        // Initialize autofocus if configured and supported by the sensor.
        // In menuconfig: Component config → Camera configuration → Enable autofocus support
        maybe_init_autofocus();
    #endif
    #endif

    asio::io_context io_context;
    Server s(io_context, SERVER_PORT);
    static CaptureCtx ctx{&io_context, &s};
    xTaskCreatePinnedToCore(capture_task, "capture", 4096, &ctx, 5, &capture_handle, 1);

    
    
    io_context.run();
}