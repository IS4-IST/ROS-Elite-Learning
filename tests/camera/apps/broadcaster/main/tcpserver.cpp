#include "details.h"

using asio::ip::tcp;

/*
This is a server that can only connect to one client (we shall see if we can manage more..)
*/
class Server{
    tcp::acceptor accept_incoming;
    tcp::socket socket;
    constexpr size_t max_length = 1024;
    char data[max_length];
    bool termination_flag = false; // there is only one context, so there is no need for this to be shared

    enum mode{
        STREAM,
        SNAPSHOT
    } working_mode;

public:
    Server(asio::io_context& context, unsigned short port) : accept_incoming{context}{

        context.post([&](){
            // if there are no exceptions then we can't execute failable code inside the constructor
            std::error_code ec;
            accept_incoming.open(tcp::v4(),ec);
            if(!ec) 
                accept_incoming..set_option(tcp::acceptor::reuse_address(true), ec);
            if(!ec) 
                accept_incoming.bind(tcp::endpoint(tcp::v4(), port), ec);
            if(!ec) 
                accept_incoming.listen(asio::socket_base::max_listen_connections, ec);

            if(!ec)
                async_accept_client();
        });
    }


    void async_accept_client(){
        accept_incoming.async_accept([&](std::error_code ec, tcp::socket socket) {
            if(!ec){
                socket = std::move(socket);
                read_incoming_size();
            } else {
                async_accept_client();
            }
        });
    }


    void read_incoming_size(){
        asio::async_read(socket, asio::buffer(data, sizeof(size_t)),[&](std::error_code ec, std::size_t length) {
            if (!ec) { 
                size_t incoming_message_size = 0;
                std::memcpy(&incoming_message_size,&data[0],sizeof(size_t));
                if(incoming_message_size > max_length){ 
                    // the incoming message size is too big
                    // drop it, scream at the client and kill the connection
                    constexpr char[] message = "the incoming message is too big";
                    // lets copy the contents to the buffer
                    std::memcpy(&data[0],message.data(),message.size()+1); //i think the plus one will copy the zero termination
                    //we can't accept right away because first we need to send message to the outside world
                    //only then do we kill the client
                    termination_flag = true;
                    warn_error(data,message.size()+1);
                } else {
                    read_remaining_message(incoming_message_size);
                }
            } else {
                async_accept_client();
            }
        });
    }


    void read_remaining_message(size_t incoming_message){
        asio::async_read(socket, asio::buffer(data, incoming_message),[&](std::error_code ec, std::size_t length) {
            if (!ec) { 
                if(strcmp("stream",data) == 0){
                    working_mode = mode::STREAM;
                    grab_frame_and_write();
                } else if(strcmp("stop",data) == 0 ){
                    termination_flag = true;
                } else if(strcmp("snapshot",data)){
                    working_mode = mode::SNAPSHOT;
                    // now we need to grab o
                    grab_frame_and_write();
                } else {
                    //unknown so we report back to client and kill connection
                    constexpr char[] message = "supported requests are \"stop\",\"stream\",\"snapshot\"";
                    std::memcpy(&data[0],message.data(),message.size()+1); //i think the plus one will copy the zero termination
                    //we can't accept right away because first we need to send message to the outside world
                    //only then do we kill the client
                    termination_flag = true;
                    warn_error(data,message.size()+1);
                    return;
                } 

            } else {
                async_accept_client();
            }
        });
    }

    void grab_frame_and_write(){
        camera_fb_t *pic = esp_camera_fb_get();
        
        // use pic->buf to access the image // do I need to copy the contents to a secondary buffer?
        // because the 

        asio::async_write(socket, asio::buffer(pic->buf,pic->len),
        [&](std::error_code ec, std::size_t length) {
             // we always return back the pointer we obtained, only then do we deal with errors 
            esp_camera_fb_return(pic);
            if (!ec) {
                if(termination_flag){
                    //kill socket
                }
            }
        });

        ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        
    }

    void warn_error(void* data,size_t length)
    {
        asio::async_write(socket_, asio::buffer(data, length),
        [this](std::error_code ec, std::size_t length) {
            if (!ec) {
                // we need to kill the socket and listen to incoming connections again
            }
        });
    }


};


extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();
    asio::io_context io_context;
    Server s(io_context, SERVER_PORT);
    io_context.run();
}