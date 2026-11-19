// p4_h264_streamer: ESP32-P4 camera node. Serves the camera as H.264 over TCP in the same
// format as the simulator's cameras (libraries/camera/stream_format.h), so the twin, ffplay
// and camera_probe connect to it exactly as to `simulator --camera`:
//
//   nc <ip> 5000 | ffplay -f h264 -fflags nobuffer -
//   ./build/library/camera_probe <ip> 5000
//   ./build/digitaltwin/digitaltwin ... --camera scene=<ip>:5000
//
// It runs the repo's own camera::Server (libraries/camera/server.cc): any number of clients,
// each starts at a keyframe, a slow client skips to the next keyframe instead of queueing.
//
// Threads:
//   capture task (core 1): waits for a camera frame, H.264-encodes it, posts it to Asio.
//   Asio thread (app_main, core 0): camera::Server, stats timer.
#include <atomic>
#include <chrono>
#include <memory>

#include <utility>   // before asio (see server.h)
#include <asio.hpp>

#include "esp_check.h"
#include "esp_eth.h"
#include "esp_eth_phy_ip101.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "camera/server.h"
#include "p4_camera.h"

namespace {

constexpr const char* TAG = "p4cam";

p4cam::Camera s_camera;
std::atomic<bool> s_force_keyframe{false};   // set on the Asio thread, consumed by the capture task
std::atomic<bool> s_time_synced{false};

struct Stats {
    std::uint32_t frames = 0, keyframes = 0;
    std::uint64_t bytes = 0;
    std::uint64_t last_id = 0;
    std::uint32_t skipped = 0;   // frame_id gaps: camera frames not encoded
    bool first = true;
    bool keyframe_wanted = false;        // a client is waiting for an IDR
    std::int64_t last_forced_us = 0;     // when the last IDR was forced
};

constexpr std::int64_t kMinForcedIdrUs = 250'000;

// ============================ Ethernet + time ===================================================

void on_eth_event(void*, esp_event_base_t, std::int32_t id, void* data) {
    if (id == ETHERNET_EVENT_CONNECTED) {
        // What the PHY negotiated with the switch/PC. "10 Mbit" or "half duplex" here means a bad
        // cable/port or a forced setting on the other side: expect heavy loss under load.
        esp_eth_handle_t eth = *static_cast<esp_eth_handle_t*>(data);
        eth_speed_t speed = ETH_SPEED_10M;
        eth_duplex_t duplex = ETH_DUPLEX_HALF;
        esp_eth_ioctl(eth, ETH_CMD_G_SPEED, &speed);
        esp_eth_ioctl(eth, ETH_CMD_G_DUPLEX_MODE, &duplex);
        ESP_LOGI(TAG, "Ethernet link up: %s Mbit/s, %s duplex; waiting for an address (DHCP)",
                 speed == ETH_SPEED_100M ? "100" : "10", duplex == ETH_DUPLEX_FULL ? "full" : "HALF");
    }
    if (id == ETHERNET_EVENT_DISCONNECTED) ESP_LOGW(TAG, "Ethernet link down (cable?)");
}

void on_got_ip(void*, esp_event_base_t, std::int32_t, void* data) {
    const auto* e = static_cast<ip_event_got_ip_t*>(data);
    ESP_LOGI(TAG, "address " IPSTR ", camera on TCP %d:  nc " IPSTR " %d | ffplay -f h264 -fflags nobuffer -",
             IP2STR(&e->ip_info.ip), CONFIG_P4CAM_PORT, IP2STR(&e->ip_info.ip), CONFIG_P4CAM_PORT);
    esp_netif_sntp_start();
}

void on_time_sync(struct timeval*) {
    if (!s_time_synced.exchange(true)) ESP_LOGI(TAG, "clock synchronised (SNTP): capture times are now Unix time");
}

esp_err_t ethernet_start() {
    // Internal EMAC + IP101 PHY. ETH_ESP32_EMAC_DEFAULT_CONFIG() already has the P4 RMII pins
    // (REF_CLK in on GPIO50); MDC/MDIO/reset/address are the Waveshare ESP32-P4-ETH wiring.
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num = 31;
    emac_config.smi_gpio.mdio_num = 52;
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 1;
    phy_config.reset_gpio_num = 51;

    esp_eth_mac_t* mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    esp_eth_phy_t* phy = esp_eth_phy_new_ip101(&phy_config);
    if (!mac || !phy) return ESP_FAIL;
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth = nullptr;
    ESP_RETURN_ON_ERROR(esp_eth_driver_install(&config, &eth), TAG, "Ethernet driver install");

    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t* netif = esp_netif_new(&netif_config);
    ESP_RETURN_ON_ERROR(esp_netif_attach(netif, esp_eth_new_netif_glue(eth)), TAG, "netif attach");

    // SNTP: the DHCP server's NTP option if it sends one, else CONFIG_P4CAM_NTP_SERVER.
    // Started once we have an address (on_got_ip).
    esp_sntp_config_t sntp = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_P4CAM_NTP_SERVER);
    sntp.start = false;
    sntp.server_from_dhcp = true;
    sntp.renew_servers_after_new_IP = true;
    sntp.index_of_first_server = 1;              // keep slot 0 for the DHCP-provided server
    sntp.ip_event_to_renew = IP_EVENT_ETH_GOT_IP;
    sntp.sync_cb = on_time_sync;
    ESP_RETURN_ON_ERROR(esp_netif_sntp_init(&sntp), TAG, "SNTP init");

    ESP_RETURN_ON_ERROR(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &on_eth_event, nullptr), TAG, "");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &on_got_ip, nullptr), TAG, "");
    return esp_eth_start(eth);
}

// ============================ capture task ======================================================

struct CaptureCtx {
    asio::io_context* io;
    camera::Server* server;
    Stats* stats;
};

void capture_task(void* arg) {
    auto* ctx = static_cast<CaptureCtx*>(arg);
    for (;;) {
        auto frame = std::make_shared<camera::EncodedFrame>();
        if (s_camera.encode_next(s_force_keyframe.exchange(false), *frame) != ESP_OK) continue;
        asio::post(*ctx->io, [ctx, frame = std::shared_ptr<const camera::EncodedFrame>(std::move(frame))] {
            ctx->server->broadcast(frame);
            // A client joined or fell behind: ask for an IDR, but at most every kMinForcedIdrUs. An IDR is
            // several times bigger than a P-frame, so forcing one per skipped frame would make a slow
            // client fall further behind (a stream of nothing but keyframes).
            Stats& st = *ctx->stats;
            st.keyframe_wanted = ctx->server->wants_keyframe() || st.keyframe_wanted;
            const std::int64_t now = esp_timer_get_time();
            if (st.keyframe_wanted && now - st.last_forced_us >= kMinForcedIdrUs) {
                s_force_keyframe = true;
                st.keyframe_wanted = false;
                st.last_forced_us = now;
            }

            Stats& s = st;
            if (!s.first && frame->stamp.frame_id > s.last_id + 1)
                s.skipped += static_cast<std::uint32_t>(frame->stamp.frame_id - s.last_id - 1);
            s.first = false;
            s.last_id = frame->stamp.frame_id;
            s.frames++;
            s.keyframes += frame->keyframe;
            s.bytes += frame->bytes.size();
        });
    }
}

void report_every(asio::steady_timer& timer, camera::Server& server, Stats& s, std::int64_t& t0, std::int64_t& enc0) {
    timer.expires_after(std::chrono::seconds(5));
    timer.async_wait([&timer, &server, &s, &t0, &enc0](std::error_code ec) {
        if (ec) return;
        const std::int64_t now = esp_timer_get_time(), enc = s_camera.encode_us_total();
        const double dt = (now - t0) / 1e6;
        ESP_LOGI(TAG, "%.1f fps | %.0f kbit/s | encode %.1f ms | keyframes %u | skipped %u | clients %u%s",
                 s.frames / dt, s.bytes * 8 / dt / 1000, s.frames ? (enc - enc0) / 1000.0 / s.frames : 0.0,
                 unsigned(s.keyframes), unsigned(s.skipped), unsigned(server.clients()),
                 s_time_synced ? "" : " | clock NOT synchronised");
        t0 = now;
        enc0 = enc;
        s.frames = s.keyframes = s.skipped = 0;
        s.bytes = 0;
        report_every(timer, server, s, t0, enc0);
    });
}

}  // namespace

extern "C" void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Camera first: the sensor is powered up and probed right after reset
    p4cam::Config config;
    config.bitrate_bps = CONFIG_P4CAM_BITRATE_KBPS * 1000u;
    config.gop = CONFIG_P4CAM_GOP;
    config.qp_min = CONFIG_P4CAM_QP_MIN;
    config.qp_max = CONFIG_P4CAM_QP_MAX;
    if (s_camera.open(config) != ESP_OK) {
        ESP_LOGE(TAG, "camera setup failed, stopping");
        return;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if ((err = ethernet_start()) != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet start failed: %s", esp_err_to_name(err));
        return;
    }

    asio::io_context io;
    camera::Server server(io);
    if (std::error_code ec = server.listen("0.0.0.0", CONFIG_P4CAM_PORT)) {
        ESP_LOGE(TAG, "%s: %s", ec.message().c_str(), server.error_detail().c_str());
        return;
    }

    static Stats stats;
    static CaptureCtx ctx{&io, &server, &stats};
    xTaskCreatePinnedToCore(capture_task, "capture", 6144, &ctx, 5, nullptr, 1);

    asio::steady_timer stats_timer(io);
    std::int64_t t0 = esp_timer_get_time(), enc0 = 0;
    report_every(stats_timer, server, stats, t0, enc0);

    vTaskPrioritySet(nullptr, 6);   // network above the capture task
    io.run();                       // never returns: the acceptor and the timer always have work
}
