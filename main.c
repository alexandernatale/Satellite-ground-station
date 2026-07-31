/*
 * Capstone — Ground Station Communications & Command System
 * ============================================================
 * Theme: Space — Satellite Ground Station (S-Band Comms / Command & Control)
 * Tailored toward: RF/Communications Systems Engineer roles
 * ============================================================
 *
 * Integrates:
 *   App 4 (base)  -- sync primitives: binary sem (ISR->task), counting sem
 *                     (downlink channel pool), mutex (telemetry_seq), plus
 *                     the H/M/L priority-inversion self-test.
 *   App 3 method  -- ISR entry timestamp + GPIO scope pulse, latency measured
 *                     on the ground-command wake path.
 *   App 2 method  -- MEASURE_WCET wrapped around ground_command_task and
 *                     telemetry_framer_task.
 *   App 5 concept -- comms_queue: subsystem tasks hand a telemetry packet to
 *                     a framer task over a FreeRTOS queue instead of just
 *                     logging in place.
 *   App 1 pattern -- Core-0 HTTP dashboard, auto-refresh table.
 *
 * ============================================================
 *  RUN MODE
 * ============================================================
 *   USE_WEBSERVER = 0 -> serial-only, no Wi-Fi (default, use while iterating)
 *   USE_WEBSERVER = 1 -> Wi-Fi + dashboard at the printed IP (flip this on
 *                        for the demo video capture)
 *
 * ============================================================
 *  FAULT INJECTION  (induced failure / degradation deliverable)
 * ============================================================
 *   USE_PI_MUTEX = 1          -> priority inheritance ON (bounded wait)
 *   USE_PI_MUTEX = 0          -> binary sem lock, unbounded inversion
 *   INDUCE_MUTEX_FROM_ISR = 1 -> ISR illegally takes shared_mux (App4 bug)
 *   Run once nominal (both defaults below), capture that as your baseline,
 *   then flip ONE at a time for the fault-injection segment of the video.
 */

#ifndef USE_WEBSERVER
#define USE_WEBSERVER 0
#endif
#ifndef USE_PI_MUTEX
#define USE_PI_MUTEX 1
#endif
#ifndef INDUCE_MUTEX_FROM_ISR
#define INDUCE_MUTEX_FROM_ISR 0
#endif

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_task_wdt.h"

#if USE_WEBSERVER
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASS ""
#endif

#define BUTTON_GPIO     GPIO_NUM_18   /* ground-command uplink trip */
#define ISR_PULSE_GPIO  GPIO_NUM_19   /* scope this for ISR->task latency */
#define BEACON_GPIO     GPIO_NUM_2    /* system-alive heartbeat LED */

static const char *TAG = "capstone";

/* ---------- Synchronization primitives (App 4) ---------- */
static SemaphoreHandle_t sig_sem;     /* binary   -- ISR -> ground_command_task */
static SemaphoreHandle_t pool_sem;    /* counting -- 3 S-band downlink channels */
static SemaphoreHandle_t shared_mux;  /* mutex    -- protects telemetry_seq */
static int telemetry_seq = 0;

/* ---------- Comms queue (App 5 concept) ---------- */
typedef struct {
    int     subsystem_id;   /* 1 = power, 2 = thermal */
    int     seq;
    int64_t stamp_us;
} comm_packet_t;

#define COMMS_Q_DEPTH 8
static QueueHandle_t comms_queue;
static volatile uint32_t frames_dropped;

/* ---------- WCET helper (App 2) ---------- */
#define MEASURE_WCET(_max_var, _body) do {                       \
    int64_t _t0 = esp_timer_get_time();                          \
    _body;                                                        \
    int64_t _dt = esp_timer_get_time() - _t0;                    \
    if ((uint64_t)_dt > (_max_var)) (_max_var) = (uint64_t)_dt;  \
} while (0)

static uint64_t wcet_gc_max_us, wcet_frame_max_us;

/* ---------- ISR latency telemetry (App 3) ---------- */
static volatile int64_t  isr_entry_time_us;
static volatile uint32_t commands_received;
static volatile int64_t  last_latency_us;
static volatile uint64_t latency_max_us;

/* ---------- Heartbeats / dashboard counters ---------- */
static volatile uint32_t hb_gc, hb_power, hb_thermal, hb_frame;
static volatile uint32_t hb_dl[4];

/* ---------- Priority-inversion self-test result (App 4) ---------- */
static volatile int64_t pi_last_wait_us;

static volatile int64_t last_edge_us;

/* ============================================================
 *  ISR -- ground command uplink trip
 * ============================================================ */
static void IRAM_ATTR button_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (now - last_edge_us < 200) return;
    last_edge_us = now;

    gpio_set_level(ISR_PULSE_GPIO, 1);
    isr_entry_time_us = now;
    commands_received++;

    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(sig_sem, &woken);

    gpio_set_level(ISR_PULSE_GPIO, 0);

#if INDUCE_MUTEX_FROM_ISR
    /* INDUCED FAILURE: mutex has no legal ISR-safe acquire path -- see App4 */
    xSemaphoreTake(shared_mux, 0);
#endif

    portYIELD_FROM_ISR(woken);
}

/* ============================================================
 *  Ground-command task -- consumes sig_sem, WCET + latency measured
 * ============================================================ */
static void ground_command_task(void *arg)
{
    for (;;) {
        if (xSemaphoreTake(sig_sem, portMAX_DELAY) == pdTRUE) {
            int64_t wake = esp_timer_get_time();
            int64_t lat = wake - isr_entry_time_us;
            last_latency_us = lat;
            if ((uint64_t)lat > latency_max_us) latency_max_us = (uint64_t)lat;

            MEASURE_WCET(wcet_gc_max_us, {
                ESP_LOGI(TAG, "[GND_CMD] uplink command #%lu ack -- executing",
                         (unsigned long)commands_received);
                vTaskDelay(pdMS_TO_TICKS(80));   /* simulated command execution */
            });

            hb_gc++;
            ESP_LOGI(TAG, "[GND_CMD] complete  lat=%lld us  max_lat=%llu us  wcet_max=%llu us",
                     (long long)lat, (unsigned long long)latency_max_us,
                     (unsigned long long)wcet_gc_max_us);
        }
    }
}

/* ============================================================
 *  Downlink channel-pool contention (App 4, unchanged)
 * ============================================================ */
static void downlink_task(void *arg)
{
    int id = (int)(uintptr_t)arg;
    for (;;) {
        if (xSemaphoreTake(pool_sem, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_LOGI(TAG, "[SAT-%d] acquired downlink channel -- transmitting", id);
            vTaskDelay(pdMS_TO_TICKS(500 + (id * 200)));
            xSemaphoreGive(pool_sem);
            hb_dl[id - 1]++;
            ESP_LOGI(TAG, "[SAT-%d] released downlink channel", id);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            ESP_LOGW(TAG, "[SAT-%d] all 3 channels busy -- backed off after 1s", id);
        }
    }
}

/* ============================================================
 *  Subsystem tasks -- mutex-protected seq, then hand a packet to the
 *  comms queue for framing (this is the new IPC pipeline piece).
 * ============================================================ */
static const char *subsystem_name(int id) { return (id == 1) ? "power" : "thermal"; }

static void subsystem_task(void *arg)
{
    int id = (int)(uintptr_t)arg;
    for (;;) {
        if (xSemaphoreTake(shared_mux, portMAX_DELAY) == pdTRUE) {
            int old = telemetry_seq;
            telemetry_seq = old + 1;
            comm_packet_t pkt = { .subsystem_id = id, .seq = telemetry_seq,
                                   .stamp_us = esp_timer_get_time() };
            ESP_LOGI(TAG, "[%s] telemetry_seq %d -> %d", subsystem_name(id), old, telemetry_seq);
            xSemaphoreGive(shared_mux);

            if (xQueueSend(comms_queue, &pkt, pdMS_TO_TICKS(50)) != pdTRUE) {
                frames_dropped++;
                ESP_LOGW(TAG, "[%s] comms_queue full -- packet dropped (total=%lu)",
                         subsystem_name(id), (unsigned long)frames_dropped);
            }
            if (id == 1) hb_power++; else hb_thermal++;
        }
        vTaskDelay(pdMS_TO_TICKS(150 + (id * 73)));
    }
}

/* ============================================================
 *  Telemetry framer -- single consumer of comms_queue (App 5 concept)
 * ============================================================ */
static void telemetry_framer_task(void *arg)
{
    comm_packet_t pkt;
    for (;;) {
        if (xQueueReceive(comms_queue, &pkt, portMAX_DELAY) == pdTRUE) {
            MEASURE_WCET(wcet_frame_max_us, {
                int64_t age_us = esp_timer_get_time() - pkt.stamp_us;
                ESP_LOGI(TAG, "[FRAME] packet from %s seq=%d age=%lld us -- frame ready",
                         subsystem_name(pkt.subsystem_id), pkt.seq, (long long)age_us);
            });
            hb_frame++;
        }
    }
}

/* ============================================================
 *  Priority-inversion self-test (App 4, H/M/L) -- runs once at boot,
 *  result feeds the dashboard. Structurally the Mars Pathfinder bug.
 * ============================================================ */
#if USE_PI_MUTEX
#define PI_LOCK_CREATE() xSemaphoreCreateMutex()
#define PI_LOCK_NAME     "MUTEX (priority inheritance ON)"
#else
#define PI_LOCK_CREATE() xSemaphoreCreateBinary()
#define PI_LOCK_NAME     "BINARY SEM (no inheritance)"
#endif

static SemaphoreHandle_t pi_lock;
#define PI_H_DELAY_MS 50
#define PI_M_DELAY_MS 100
#define PI_L_ITERS    20000000UL
#define PI_M_ITERS    40000000UL
static volatile uint32_t pi_sink;

static void pi_burn(uint32_t iters)
{
    uint32_t x = pi_sink ? pi_sink : 1u;
    for (uint32_t i = 0; i < iters; i++) { x ^= (x << 5); x += i; }
    pi_sink = x;
}

static void pi_low_task(void *arg)
{
    xSemaphoreTake(pi_lock, portMAX_DELAY);
    ESP_LOGI(TAG, "[PI][L] took lock -- entering CPU-bound section");
    pi_burn(PI_L_ITERS);
    xSemaphoreGive(pi_lock);
    ESP_LOGI(TAG, "[PI][L] released lock");
    vTaskDelete(NULL);
}

static void pi_med_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(PI_M_DELAY_MS));
    ESP_LOGI(TAG, "[PI][M] burning CPU (takes no lock)");
    pi_burn(PI_M_ITERS);
    ESP_LOGI(TAG, "[PI][M] done");
    vTaskDelete(NULL);
}

static void pi_high_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(PI_H_DELAY_MS));
    int64_t t_block = esp_timer_get_time();
    xSemaphoreTake(pi_lock, portMAX_DELAY);
    int64_t wait = esp_timer_get_time() - t_block;
    pi_last_wait_us = wait;
    ESP_LOGW(TAG, "[PI][H] ACQUIRED -- waited %lld us  [lock=%s]",
             (long long)wait, PI_LOCK_NAME);
    xSemaphoreGive(pi_lock);
    vTaskDelete(NULL);
}

static void start_inversion_demo(void)
{
    pi_lock = PI_LOCK_CREATE();
#if !USE_PI_MUTEX
    xSemaphoreGive(pi_lock);
#endif
    xTaskCreatePinnedToCore(pi_high_task, "H", 4096, NULL, 15, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(pi_med_task,  "M", 4096, NULL, 10, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(pi_low_task,  "L", 4096, NULL,  5, NULL, APP_CPU_NUM);
}

/* ============================================================
 *  Beacon -- system-alive heartbeat LED
 * ============================================================ */
static void beacon_task(void *arg)
{
    gpio_reset_pin(BEACON_GPIO);
    gpio_set_direction(BEACON_GPIO, GPIO_MODE_OUTPUT);
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000);
    static bool on = false;
    for (;;) {
        on = !on;
        gpio_set_level(BEACON_GPIO, on);
        vTaskDelayUntil(&last, period);
    }
}

#if USE_WEBSERVER
/* ============================================================
 *  WEB DASHBOARD  (USE_WEBSERVER = 1)
 * ============================================================ */
static esp_err_t handle_root(httpd_req_t *req)
{
    char buf[3072];
    int avail  = (int)uxSemaphoreGetCount(pool_sem);
    int qdepth = (int)uxQueueMessagesWaiting(comms_queue);
    int n = snprintf(buf, sizeof(buf),
        "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">"
        "<meta http-equiv=\"refresh\" content=\"1\">"
        "<title>Ground Station -- Comms &amp; Command</title>"
        "<style>"
        "body{font-family:-apple-system,sans-serif;background:#06111c;color:#cfe8ff;padding:1.5rem;}"
        "h1{color:#4fd1ff;border-bottom:3px solid #4fd1ff;display:inline-block;padding-bottom:4px;}"
        "table{border-collapse:collapse;margin:1rem 0;width:100%%;max-width:640px;}"
        "th{background:#0d2233;color:#4fd1ff;padding:6px 12px;text-align:left;font-size:12px;text-transform:uppercase;}"
        "td{padding:5px 12px;border-bottom:1px solid #123249;}"
        "td.num{font-variant-numeric:tabular-nums;font-weight:700;color:#7fe0a7;}"
        "</style></head><body>"
        "<h1>Ground Station -- S-Band Comms &amp; Command</h1>"
        "<table><tr><th>Metric</th><th>Value</th></tr>"
        "<tr><td>Ground-command latency (last / max)</td><td class=\"num\">%lld / %llu us</td></tr>"
        "<tr><td>Ground-command WCET max</td><td class=\"num\">%llu us</td></tr>"
        "<tr><td>Downlink channels available</td><td class=\"num\">%d / 3</td></tr>"
        "<tr><td>Comms queue depth</td><td class=\"num\">%d / %d</td></tr>"
        "<tr><td>Frames dropped</td><td class=\"num\">%lu</td></tr>"
        "<tr><td>Framer WCET max</td><td class=\"num\">%llu us</td></tr>"
        "<tr><td>Telemetry seq</td><td class=\"num\">%d</td></tr>"
        "<tr><td>Heartbeats gc/pwr/thm/frame</td><td class=\"num\">%lu / %lu / %lu / %lu</td></tr>"
        "<tr><td>Downlink heartbeats SAT1-4</td><td class=\"num\">%lu / %lu / %lu / %lu</td></tr>"
        "<tr><td>PI self-test H-wait</td><td class=\"num\">%lld us</td></tr>"
        "<tr><td>Fault injection</td><td class=\"num\">lock=%s  ISR-mutex=%s</td></tr>"
        "</table><p>Auto-refresh 1s.</p></body></html>",
        (long long)last_latency_us, (unsigned long long)latency_max_us,
        (unsigned long long)wcet_gc_max_us,
        avail, qdepth, COMMS_Q_DEPTH,
        (unsigned long)frames_dropped,
        (unsigned long long)wcet_frame_max_us,
        telemetry_seq,
        (unsigned long)hb_gc, (unsigned long)hb_power, (unsigned long)hb_thermal, (unsigned long)hb_frame,
        (unsigned long)hb_dl[0], (unsigned long)hb_dl[1], (unsigned long)hb_dl[2], (unsigned long)hb_dl[3],
        (long long)pi_last_wait_us,
        USE_PI_MUTEX ? "mutex(ON)" : "binsem(OFF)",
        INDUCE_MUTEX_FROM_ISR ? "ON" : "OFF");

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.core_id = 0;
    cfg.task_priority = 5;
    cfg.stack_size = 8192;
    httpd_handle_t s = NULL;
    if (httpd_start(&s, &cfg) == ESP_OK) {
        httpd_uri_t root = { .uri="/", .method=HTTP_GET, .handler=handle_root, .user_ctx=NULL };
        httpd_register_uri_handler(s, &root);
    }
    return s;
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
        start_webserver();
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t cfg = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS,
                                   .threshold.authmode = WIFI_AUTH_OPEN } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}

#else  /* USE_WEBSERVER == 0 */
/* ============================================================
 *  TERMINAL MONITOR  (USE_WEBSERVER = 0)
 * ============================================================ */
static void task_monitor(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000);
    for (;;) {
        printf("\n=== Ground Station -- Comms & Command ===\n");
        printf("gc_lat(last/max)=%lld/%llu us  gc_wcet=%llu us\n",
               (long long)last_latency_us, (unsigned long long)latency_max_us,
               (unsigned long long)wcet_gc_max_us);
        printf("channels_avail=%d/3  q_depth=%d/%d  dropped=%lu  frame_wcet=%llu us\n",
               (int)uxSemaphoreGetCount(pool_sem), (int)uxQueueMessagesWaiting(comms_queue),
               COMMS_Q_DEPTH, (unsigned long)frames_dropped, (unsigned long long)wcet_frame_max_us);
        printf("telemetry_seq=%d  hb: gc=%lu pwr=%lu thm=%lu frame=%lu\n",
               telemetry_seq, (unsigned long)hb_gc, (unsigned long)hb_power,
               (unsigned long)hb_thermal, (unsigned long)hb_frame);
        printf("hb_dl SAT1-4: %lu %lu %lu %lu   PI H-wait=%lld us  [lock=%s  ISR-mutex=%s]\n",
               (unsigned long)hb_dl[0], (unsigned long)hb_dl[1],
               (unsigned long)hb_dl[2], (unsigned long)hb_dl[3],
               (long long)pi_last_wait_us,
               USE_PI_MUTEX ? "mutex(ON)" : "binsem(OFF)",
               INDUCE_MUTEX_FROM_ISR ? "ON" : "OFF");
        vTaskDelayUntil(&last, period);
    }
}
#endif /* USE_WEBSERVER */

/* ---------- app_main ---------- */
void app_main(void)
{
    esp_task_wdt_reconfigure(&(esp_task_wdt_config_t){.timeout_ms = 10000, .idle_core_mask = 0, .trigger_panic = false });
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "==== Capstone [Ground Station -- Comms & Command] starting ====");
    ESP_LOGI(TAG, "Lock mode: %s (USE_PI_MUTEX=%d)  Induced ISR-mutex: %d",
             PI_LOCK_NAME, USE_PI_MUTEX, INDUCE_MUTEX_FROM_ISR);

    sig_sem      = xSemaphoreCreateBinary();
    pool_sem     = xSemaphoreCreateCounting(3, 3);
    shared_mux   = xSemaphoreCreateMutex();
    comms_queue  = xQueueCreate(COMMS_Q_DEPTH, sizeof(comm_packet_t));

    gpio_config_t btn_cfg = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&btn_cfg);

    gpio_config_t pulse_cfg = {
        .pin_bit_mask = 1ULL << ISR_PULSE_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pulse_cfg);
    gpio_set_level(ISR_PULSE_GPIO, 0);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr, NULL);

    xTaskCreatePinnedToCore(ground_command_task, "ground_cmd", 4096, NULL, 12, NULL, APP_CPU_NUM);

    for (int i = 1; i <= 4; i++) {
        xTaskCreatePinnedToCore(downlink_task, "downlink", 4096,
                                (void*)(uintptr_t)i, 5, NULL, APP_CPU_NUM);
    }

    xTaskCreatePinnedToCore(subsystem_task, "power_subsys",   4096, (void*)1, 8, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(subsystem_task, "thermal_subsys", 4096, (void*)2, 8, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(telemetry_framer_task, "framer",  4096, NULL,     6, NULL, APP_CPU_NUM);

    xTaskCreatePinnedToCore(beacon_task, "beacon", 2048, NULL, 3, NULL, APP_CPU_NUM);

#if USE_WEBSERVER
    ESP_LOGI(TAG, "Output mode: WEB DASHBOARD -- open the printed IP");
    wifi_init_sta();
#else
    ESP_LOGI(TAG, "Output mode: TERMINAL MONITOR -- no Wi-Fi, serial only");
    xTaskCreatePinnedToCore(task_monitor, "task_monitor", 4096, NULL, 1, NULL, PRO_CPU_NUM);
#endif

    /* For the cleanest H-wait number, this runs alongside everything else --
     * that's realistic (a self-test running under real system load), but if
     * your numbers look noisy, temporarily comment the blocks above. */
    start_inversion_demo();
}