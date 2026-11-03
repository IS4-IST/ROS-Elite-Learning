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
#include "esp_timer.h"

static const char *TAG = "tcp_server";

#define AP_SSID             "esp32cam"
#define AP_PASSWORD         "changeme123"
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

// rubish overloads because we dont' want exceptions
template void throw_exception<asio::system_error>(const asio::system_error& ASIO_SOURCE_LOCATION_PARAM);
template void throw_exception<std::bad_alloc>(const std::bad_alloc& ASIO_SOURCE_LOCATION_PARAM);
template void throw_exception<asio::invalid_service_owner>(const asio::invalid_service_owner& ASIO_SOURCE_LOCATION_PARAM);
template void throw_exception<asio::service_already_exists>(const asio::service_already_exists& ASIO_SOURCE_LOCATION_PARAM);
template void throw_exception<asio::ip::bad_address_cast>(const asio::ip::bad_address_cast& ASIO_SOURCE_LOCATION_PARAM);
} 

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


#include "sdkconfig.h"

#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"

#if defined(CONFIG_CAMERA_AF_SUPPORT) && CONFIG_CAMERA_AF_SUPPORT
#include "esp_camera_af.h"
#endif

#define BOARD_ESP32CAM_AITHINKER 1

#include "camera_pinout.h"

#if ESP_CAMERA_SUPPORTED
camera_config_t* get_config(){
    static camera_config_t camera_config;
    camera_config.pin_pwdn = CAM_PIN_PWDN;
    camera_config.pin_reset = CAM_PIN_RESET;
    camera_config.pin_xclk = CAM_PIN_XCLK;
    camera_config.pin_sccb_sda = CAM_PIN_SIOD;
    camera_config.pin_sccb_scl = CAM_PIN_SIOC;

    camera_config.pin_d7 = CAM_PIN_D7;
    camera_config.pin_d6 = CAM_PIN_D6;
    camera_config.pin_d5 = CAM_PIN_D5;
    camera_config.pin_d4 = CAM_PIN_D4;
    camera_config.pin_d3 = CAM_PIN_D3;
    camera_config.pin_d2 = CAM_PIN_D2;
    camera_config.pin_d1 = CAM_PIN_D1;
    camera_config.pin_d0 = CAM_PIN_D0;
    camera_config.pin_vsync = CAM_PIN_VSYNC;
    camera_config.pin_href = CAM_PIN_HREF;
    camera_config.pin_pclk = CAM_PIN_PCLK;

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    camera_config.xclk_freq_hz = 10000000; //16
    camera_config.ledc_timer = LEDC_TIMER_0;
    camera_config.ledc_channel = LEDC_CHANNEL_0;

    //camera_config.jpeg_buffer_size = 200000;

    camera_config.pixel_format = PIXFORMAT_JPEG; //YUV422,GRAYSCALE,RGB565,JPEG
    camera_config.frame_size = FRAMESIZE_HD;    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    camera_config.jpeg_quality = 7; //0-63, for OV series camera sensors, lower number means higher quality
    camera_config.fb_count = 3;       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    camera_config.fb_location = CAMERA_FB_IN_PSRAM;
    camera_config.grab_mode = CAMERA_GRAB_LATEST;
    camera_config.jpeg_buffer_size = 250000; //Set to a custom size in bytes for JPEG mode, or 0 to use the default size

    return &camera_config;
}

static esp_err_t init_camera(void)
{
    //initialize the camera
    auto config = get_config();
    esp_err_t err = esp_camera_init(config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

#if defined(CONFIG_CAMERA_AF_SUPPORT) && CONFIG_CAMERA_AF_SUPPORT
static void maybe_init_autofocus(void)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGW(TAG, "AF: no sensor handle");
        return;
    }

    if (!esp_camera_af_is_supported(s)) {
        ESP_LOGI(TAG, "AF: not supported by this sensor");
        return;
    }

    esp_camera_af_config_t af_cfg = {
        .mode = ESP_CAMERA_AF_MODE_AUTO,
        .timeout_ms = CONFIG_CAMERA_AF_DEFAULT_TIMEOUT_MS,
    };

    esp_err_t ret = esp_camera_af_init(s, &af_cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AF init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "AF initialized (AUTO mode)");
}
#endif
#endif