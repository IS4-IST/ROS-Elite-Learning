/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 *
 * Espressif's asio tcp_echo_server example, adapted:
 *  - Wi-Fi SoftAP instead of example_connect()
 *  - built without C++ exceptions (ASIO_NO_EXCEPTIONS), so error_code overloads are used
 *
 * Join the "esp32cam" Wi-Fi from the PC, then:  nc 192.168.4.1 3333
 */
#include "asio.hpp"
#include <string>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <new>
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "tcp_server";

#define AP_SSID             "esp32cam"
#define AP_PASSWORD         "changeme123"   // min. 8 chars for WPA2. CHANGE IT.
#define AP_CHANNEL          6
#define AP_MAX_CONNECTIONS  2
#define SERVER_PORT         3333

namespace asio::detail {
template <typename Exception>
void throw_exception(const Exception& e ASIO_SOURCE_LOCATION_PARAM)
{
    ESP_LOGE("asio", "fatal error: %s", e.what());
    abort();
}

// Make sure the versions asio's own compiled code (asio.cpp) needs exist.
// If the linker reports another missing throw_exception<T>, add a line for T here.
template void throw_exception<asio::system_error>(const asio::system_error& ASIO_SOURCE_LOCATION_PARAM);
template void throw_exception<std::bad_alloc>(const std::bad_alloc& ASIO_SOURCE_LOCATION_PARAM);
template void throw_exception<asio::invalid_service_owner>(const asio::invalid_service_owner& ASIO_SOURCE_LOCATION_PARAM);
template void throw_exception<asio::service_already_exists>(const asio::service_already_exists& ASIO_SOURCE_LOCATION_PARAM);

} // namespace asio::detail

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        auto *e = static_cast<wifi_event_ap_staconnected_t*>(event_data);
        ESP_LOGI(TAG, "station " MACSTR " joined, AID=%d", MAC2STR(e->mac), e->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        auto *e = static_cast<wifi_event_ap_stadisconnected_t*>(event_data);
        ESP_LOGI(TAG, "station " MACSTR " left, reason=%d", MAC2STR(e->mac), e->reason);
    }
}

static void wifi_init_softap()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();                 // ESP32 = 192.168.4.1, runs DHCP

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, nullptr, nullptr));

    // C++: fill field by field (designated initializers must follow declaration order)
    wifi_config_t wifi_config = {};
    strncpy(reinterpret_cast<char*>(wifi_config.ap.ssid), AP_SSID, sizeof(wifi_config.ap.ssid));
    strncpy(reinterpret_cast<char*>(wifi_config.ap.password), AP_PASSWORD, sizeof(wifi_config.ap.password));
    wifi_config.ap.ssid_len       = strlen(AP_SSID);
    wifi_config.ap.channel        = AP_CHANNEL;
    wifi_config.ap.max_connection = AP_MAX_CONNECTIONS;
    wifi_config.ap.authmode       = strlen(AP_PASSWORD) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    wifi_config.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "SoftAP \"%s\" up, connect to 192.168.4.1:%d", AP_SSID, SERVER_PORT);
}

using asio::ip::tcp;

class session
    : public std::enable_shared_from_this<session> {
public:
    session(tcp::socket socket)
        : socket_(std::move(socket))
    {
    }

    void start()
    {
        do_read();
    }

private:
    void do_read()
    {
        auto self(shared_from_this());
        // read at most max_length - 1 so there is room for the terminating 0 below
        socket_.async_read_some(asio::buffer(data_, max_length - 1),
        [this, self](std::error_code ec, std::size_t length) {
            if (!ec) {
                data_[length] = 0;
                std::cout << data_ << std::endl;
                do_write(length);
            }
        });
    }

    void do_write(std::size_t length)
    {
        auto self(shared_from_this());
        asio::async_write(socket_, asio::buffer(data_, length),
        [this, self](std::error_code ec, std::size_t length) {
            if (!ec) {
                do_read();
            }
        });
    }

    tcp::socket socket_;
    enum { max_length = 1024 };
    char data_[max_length];
};

class server {
public:
    // Built step by step with error_code overloads: the one-line
    // acceptor_(io_context, endpoint) constructor would abort on failure.
    server(asio::io_context &io_context, unsigned short port)
        : acceptor_(io_context)
    {
        std::error_code ec;
        acceptor_.open(tcp::v4(), ec);
        if (!ec) acceptor_.set_option(tcp::acceptor::reuse_address(true), ec);
        if (!ec) acceptor_.bind(tcp::endpoint(tcp::v4(), port), ec);
        if (!ec) acceptor_.listen(asio::socket_base::max_listen_connections, ec);
        if (ec) {
            ESP_LOGE(TAG, "acceptor setup failed: %s", ec.message().c_str());
            return;
        }
        ESP_LOGI(TAG, "listening on port %u", port);
        do_accept();
    }

private:
    void do_accept()
    {
        acceptor_.async_accept(
        [this](std::error_code ec, tcp::socket socket) {
            if (!ec) {
                std::make_shared<session>(std::move(socket))->start();
            }

            do_accept();
        });
    }

    tcp::acceptor acceptor_;
};


extern "C" void app_main(void)
{
    // NVS is used by the Wi-Fi driver to store calibration data
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();     // replaces esp_netif_init + event loop + example_connect()

    asio::io_context io_context;

    server s(io_context, SERVER_PORT);

    std::cout << "ASIO engine is up and running" << std::endl;

    io_context.run();       // runs forever, handling all async operations
}