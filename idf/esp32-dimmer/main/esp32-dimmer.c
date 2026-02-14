#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "rom/ets_sys.h" // For ets_delay_us
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include <string.h>

#define PIN_ZCD_INPUT GPIO_NUM_23   // Input from Zero Cross Detector
#define PIN_TRIAC_GATE GPIO_NUM_22  // Output to TRIAC Gate
#define PIN_SWITCH GPIO_NUM_21      // Physical toggle switch (active low)
#define TRIAC_PULSE_US 600          // Duration of the trigger pulse in microseconds (shorter reduces heating/flicker)
#define MAINS_FREQ_DEFAULT 60       // Fallback frequency if detection fails
#define TIMER_RESOLUTION_HZ 1000000 // 1MHz resolution (1 tick = 1us)
#define SWITCH_DEBOUNCE_MS 50      // Debounce time for switch
#define SWITCH_DOUBLE_TAP_MS 400   // Max time between taps to consider double-tap
#define ZCD_MIN_US 2000            // Ignore ZCD intervals shorter than this (noise)
#define ZCD_MAX_US 15000           // Ignore ZCD intervals longer than this
#define ZCD_TOLERANCE_PERCENT 8    // Acceptable deviation from expected half-period (percent)
#define ZCD_GOOD_REQUIRED 2       // Consecutive good cycles required to accept a measured change

static const char *TAG = "DIMMER";

typedef struct
{
    uint32_t period_us;     // Calculated period of half-cycle
    float frequency_hz;     // Calculated Frequency
    float power_percent;    // Target Power (0.0 to 100.0)
    float bias_percent;     // Hardware offset bias (0.0 to 100.0)
    uint64_t last_zcd_time; // Timestamp of last ZCD interrupt
    float last_nonzero_power; // Remember last non-zero power for toggle restore
} dimmer_state_t;

// ZCD event: timestamp + level (rising/falling). We compute midpoint
// between paired edges to get accurate zero-cross timing when the ZCD
// produces a pulse rather than a sharp edge.
typedef struct {
    uint64_t time;
    uint8_t level;
} zcd_event_t;

volatile dimmer_state_t d_state = {
    .period_us = 8333,
    .frequency_hz = MAINS_FREQ_DEFAULT,
    .power_percent = 0.0, // Start off
    .bias_percent = 17.0,  // Example: 5% offset
    .last_zcd_time = 0,
    .last_nonzero_power = 100.0f};

static QueueHandle_t button_queue = NULL;


gptimer_handle_t gptimer = NULL;
gptimer_handle_t pulse_timer = NULL;
static QueueHandle_t zcd_queue = NULL;
// Scheduled on/off timer
static TimerHandle_t schedule_timer = NULL;
static uint8_t scheduled_action = 0xFF; // 0=off,1=on,0xFF=none

// ---------------- BLE CONFIG ----------------
#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

// UUIDs (128-bit) - used by web client. Keep as byte arrays for esp-idf APIs.
#define SERVICE_UUID {0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f, 0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f}
#define CHAR_UUID {0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7, 0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe}

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = (uint8_t[]){SERVICE_UUID},
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void IRAM_ATTR switch_isr_handler(void *arg);
static void button_task(void *arg);
static esp_gatt_if_t g_gatts_if = 0;
static uint16_t g_conn_id = 0;
static uint16_t g_char_handle = 0;
static bool g_connected = false;

// NVS helpers to persist power and bias
static void load_settings(void);
static void save_settings(void);

// Timer Alarm ISR
static bool IRAM_ATTR triac_pulse_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    gpio_set_level(PIN_TRIAC_GATE, 0);
    gptimer_stop(timer);
    return false;
}

static bool IRAM_ATTR triac_fire_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    // Set gate high and arm the short pulse timer to clear it. Keep this callback minimal and non-blocking.
    gpio_set_level(PIN_TRIAC_GATE, 1);

    if (pulse_timer) {
        gptimer_alarm_config_t pulse_alarm = {
            .alarm_count = TRIAC_PULSE_US,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = false,
        };
        gptimer_set_raw_count(pulse_timer, 0);
        gptimer_set_alarm_action(pulse_timer, &pulse_alarm);
        gptimer_start(pulse_timer);
    }

    gptimer_stop(timer);
    return false;
}

// Callback run in Timer/Task context when a scheduled on/off fires
static void schedule_timer_cb(TimerHandle_t xTimer)
{
    if (scheduled_action == 1) {
        float restore = d_state.last_nonzero_power;
        if (restore < 1.0f)
            restore = 50.0f;
        d_state.power_percent = restore;
        d_state.last_nonzero_power = restore;
        ESP_LOGI(TAG, "Scheduled action: TURN ON -> %.1f%%", d_state.power_percent);
    } else if (scheduled_action == 0) {
        if (d_state.power_percent > 1.0f)
            d_state.last_nonzero_power = d_state.power_percent;
        d_state.power_percent = 0.0f;
        ESP_LOGI(TAG, "Scheduled action: TURN OFF");
    } else {
        ESP_LOGI(TAG, "Scheduled action: unknown (%d)", scheduled_action);
    }
    save_settings();
    scheduled_action = 0xFF;
    if (schedule_timer) {
        xTimerDelete(schedule_timer, 0);
        schedule_timer = NULL;
    }
}

// GPIO ISR
static void IRAM_ATTR zcd_isr_handler(void *arg)
{
    uint64_t current_time = esp_timer_get_time();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (zcd_queue)
    {
        zcd_event_t ev = { .time = current_time, .level = (uint8_t)gpio_get_level(PIN_ZCD_INPUT) };
        xQueueSendFromISR(zcd_queue, &ev, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR();
    }
}

// Switch ISR (button pressed -> falling edge)
static void IRAM_ATTR switch_isr_handler(void *arg)
{
    // Only enqueue when the level is actually low (pressed)
    if (gpio_get_level(PIN_SWITCH) != 0)
        return;

    uint64_t current_time = esp_timer_get_time();

    // Simple ISR debounce: ignore events closer than SWITCH_DEBOUNCE_MS
    static uint64_t last_time = 0;
    if (current_time - last_time < (uint64_t)SWITCH_DEBOUNCE_MS * 1000ULL)
        return;
    last_time = current_time;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (button_queue)
    {
        xQueueSendFromISR(button_queue, &current_time, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR();
    }
}

// Button task: debounce, detect single vs double tap
static void button_task(void *arg)
{
    uint64_t ts = 0;

    for (;;)
    {
        if (xQueueReceive(button_queue, &ts, portMAX_DELAY) != pdTRUE)
            continue;

        // Basic debounce: wait and flush any bounces
        vTaskDelay(pdMS_TO_TICKS(SWITCH_DEBOUNCE_MS));
        uint64_t tmp;
        while (xQueueReceive(button_queue, &tmp, 0) == pdTRUE)
        {
            ts = tmp; // take latest during bounce
        }

        // Wait for a possible second tap within DOUBLE_TAP_MS
        uint64_t second_ts;
        TickType_t wait_ticks = pdMS_TO_TICKS(SWITCH_DOUBLE_TAP_MS);
        if (xQueueReceive(button_queue, &second_ts, wait_ticks) == pdTRUE)
        {
            // Debounce second
            vTaskDelay(pdMS_TO_TICKS(SWITCH_DEBOUNCE_MS));
            // Double-tap action: set power to 100%
            d_state.power_percent = 100.0f;
            d_state.last_nonzero_power = 100.0f;
            save_settings();
            ESP_LOGI(TAG, "Switch double-tap: set power 100%%");
        }
        else
        {
            // Single tap: toggle (off <-> restore last non-zero)
            if (d_state.power_percent > 1.0f)
            {
                d_state.last_nonzero_power = d_state.power_percent;
                d_state.power_percent = 0.0f;
                save_settings();
                ESP_LOGI(TAG, "Switch single-tap: OFF");
            }
            else
            {
                float restore = d_state.last_nonzero_power;
                if (restore < 1.0f)
                    restore = 50.0f;
                d_state.power_percent = restore;
                save_settings();
                ESP_LOGI(TAG, "Switch single-tap: restore %.1f%%", d_state.power_percent);
            }
        }
    }
}

// Process ZCD events and arm timer / fire triac as needed
static void zcd_task(void *arg)
{
    zcd_event_t ev;
    uint64_t last_edge_time = 0;
    int last_edge_level = -1;
    uint64_t last_zero_time = 0;
    int consecutive_good = 0;

    for (;;)
    {
        if (xQueueReceive(zcd_queue, &ev, portMAX_DELAY) != pdTRUE)
            continue;

        // Pair edges to compute the midpoint of the ZCD pulse (more accurate zero crossing).
        if (last_edge_time == 0) {
            last_edge_time = ev.time;
            last_edge_level = ev.level;
            continue;
        }

        // If we see the same level again, update the stored edge time (debounce-like)
        if (ev.level == last_edge_level) {
            last_edge_time = ev.time;
            continue;
        }

        // We have a pair of opposite edges: compute midpoint
        uint64_t zero_time = (last_edge_time + ev.time) / 2ULL;
        uint32_t diff = (uint32_t)(zero_time - last_zero_time);
        last_zero_time = zero_time;
        d_state.last_zcd_time = zero_time;

        // Reset stored edge so next pulse starts fresh
        last_edge_time = 0;
        last_edge_level = -1;

        // Ignore obviously-bad intervals (likely noise / bounces)
        if (diff < ZCD_MIN_US || diff > ZCD_MAX_US) {
            continue;
        }

        // Keep using a fixed mains half-period for timing; only accept measured changes
        // after several consecutive valid cycles to avoid sudden sync jumps.
        uint32_t expected_half_us = (uint32_t)(1000000UL / (MAINS_FREQ_DEFAULT * 2UL));
        uint32_t tol_us = (expected_half_us * ZCD_TOLERANCE_PERCENT) / 100U;
        if (diff >= (expected_half_us > tol_us ? expected_half_us - tol_us : 0) && diff <= expected_half_us + tol_us) {
            consecutive_good++;
        } else {
            consecutive_good = 0;
        }

        // If we have not reached the required number of good cycles yet, do not change period.
        d_state.period_us = expected_half_us;

        float target_power = d_state.power_percent;
        if (target_power <= 1.0f)
        {
            gptimer_stop(gptimer);
            continue;
        }

        if (target_power >= 99.0f)
        {
            // Fire immediately but use the pulse_timer to clear the gate (non-blocking)
            gpio_set_level(PIN_TRIAC_GATE, 1);
            if (pulse_timer) {
                gptimer_alarm_config_t pulse_alarm = {
                    .alarm_count = TRIAC_PULSE_US,
                    .reload_count = 0,
                    .flags.auto_reload_on_alarm = false,
                };
                gptimer_set_raw_count(pulse_timer, 0);
                gptimer_set_alarm_action(pulse_timer, &pulse_alarm);
                gptimer_start(pulse_timer);
            }
            gptimer_stop(gptimer);
            continue;
        }

        float delay_factor = (100.0f - target_power) / 100.0f;
        int32_t fire_delay = (int32_t)(d_state.period_us * delay_factor);
        int32_t bias_us = (int32_t)(d_state.period_us * (d_state.bias_percent / 100.0f));
        fire_delay -= bias_us;

        if (fire_delay < 100)
            fire_delay = 100;
        if (fire_delay > d_state.period_us - 200)
        {
            gptimer_stop(gptimer);
            continue;
        }

        gptimer_alarm_config_t alarm_config = {
            .alarm_count = fire_delay,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = false,
        };

        gptimer_set_raw_count(gptimer, 0);
        gptimer_set_alarm_action(gptimer, &alarm_config);
        gptimer_start(gptimer);
    }
}

void setup_gpio()
{
    // Output (Triac Gate)
    gpio_config_t io_conf_out = {
        .pin_bit_mask = (1ULL << PIN_TRIAC_GATE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf_out);
    gpio_set_level(PIN_TRIAC_GATE, 0);

    // Input (ZCD)
    gpio_config_t io_conf_in = {
        .pin_bit_mask = (1ULL << PIN_ZCD_INPUT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0, // Assume Open Collector Opto
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_ANYEDGE // Trigger on any edge; we'll use midpoint of pulse
    };
    gpio_config(&io_conf_in);

    // Install ISR Service
    gpio_install_isr_service(0);
    // Create queue and zcd processing task
    zcd_queue = xQueueCreate(10, sizeof(zcd_event_t));
    if (zcd_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create ZCD queue");
    }
    else
    {
        xTaskCreate(zcd_task, "zcd_task", 3072, NULL, configMAX_PRIORITIES - 1, NULL);
    }
    gpio_isr_handler_add(PIN_ZCD_INPUT, zcd_isr_handler, NULL);

    // Configure physical switch (active low, pull-up)
    gpio_config_t io_conf_sw = {
        .pin_bit_mask = (1ULL << PIN_SWITCH),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_NEGEDGE // falling edge = pressed
    };
    gpio_config(&io_conf_sw);

    // Create button queue and task
    button_queue = xQueueCreate(5, sizeof(uint64_t));
    if (button_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create button queue");
    } else {
        xTaskCreate(button_task, "button_task", 2048, NULL, configMAX_PRIORITIES - 2, NULL);
    }

    gpio_isr_handler_add(PIN_SWITCH, switch_isr_handler, NULL);
}

void setup_timer()
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ, // 1us tick
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &pulse_timer));

    gptimer_event_callbacks_t fire_cbs = {
        .on_alarm = triac_fire_cb,
    };
    gptimer_event_callbacks_t pulse_cbs = {
        .on_alarm = triac_pulse_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &fire_cbs, NULL));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(pulse_timer, &pulse_cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_enable(pulse_timer));
}

// Load persisted settings from NVS
static void load_settings(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("dimmer", NVS_READONLY, &nvs);
    if (err == ESP_OK)
    {
        int32_t p = 0, b = 0;
        if (nvs_get_i32(nvs, "power", &p) == ESP_OK)
        {
            if (p < 0) p = 0;
            if (p > 100) p = 100;
            d_state.power_percent = (float)p;
            if (p > 1)
                d_state.last_nonzero_power = (float)p;
            ESP_LOGI(TAG, "Loaded power %% from NVS: %d", p);
        }
        if (nvs_get_i32(nvs, "bias", &b) == ESP_OK)
        {
            if (b < 0) b = 0;
            if (b > 100) b = 100;
            d_state.bias_percent = (float)b;
            ESP_LOGI(TAG, "Loaded bias %% from NVS: %d", b);
        }
        nvs_close(nvs);
    }
    else
    {
        ESP_LOGI(TAG, "No stored settings in NVS or NVS open failed: %s", esp_err_to_name(err));
    }
}

// Save current settings to NVS
static void save_settings(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("dimmer", NVS_READWRITE, &nvs);
    if (err == ESP_OK)
    {
        int32_t p = (int32_t)(d_state.power_percent + 0.5f);
        int32_t b = (int32_t)(d_state.bias_percent + 0.5f);
        nvs_set_i32(nvs, "power", p);
        nvs_set_i32(nvs, "bias", b);
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI(TAG, "Saved settings to NVS: power=%d bias=%d", p, b);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to open NVS for save: %s", esp_err_to_name(err));
    }
}

// BLE handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
    {
        esp_ble_gap_set_device_name("ESP32_DIMMER");
        esp_ble_gap_config_adv_data(&adv_data);

        // Create Service
        esp_gatt_srvc_id_t service_id;
        service_id.is_primary = true;
        service_id.id.inst_id = 0x00;
        service_id.id.uuid.len = ESP_UUID_LEN_128;
        uint8_t svc_uuid128[16] = SERVICE_UUID;
        memcpy(service_id.id.uuid.uuid.uuid128, svc_uuid128, 16);

        esp_ble_gatts_create_service(gatts_if, &service_id, 4);
        break;
    }

    case ESP_GATTS_CREATE_EVT:
    {
        uint8_t char_uuid128[16] = CHAR_UUID;
        esp_bt_uuid_t char_uuid;
        char_uuid.len = ESP_UUID_LEN_128;
        memcpy(char_uuid.uuid.uuid128, char_uuid128, 16);

        esp_ble_gatts_add_char(param->create.service_handle, &char_uuid,
                               ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                               NULL, NULL);

        esp_ble_gatts_start_service(param->create.service_handle);
        break;
    }

    case ESP_GATTS_ADD_CHAR_EVT: {
        // Save the attribute handle for notifications
        g_char_handle = param->add_char.attr_handle;

        // Add Client Characteristic Configuration Descriptor (CCCD) so client can enable notifications
        esp_bt_uuid_t descr_uuid;
        descr_uuid.len = ESP_UUID_LEN_16;
        descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_ble_gatts_add_char_descr(param->add_char.service_handle, &descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        break;
    }

    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
        // descriptor added
        break;
    }

    case ESP_GATTS_WRITE_EVT:
    {
        // Special BLE scheduling command format:
        // [0] = 0xFE, [1]=action(0=off,1=on,2=cancel), [2..5]=delay seconds (big-endian uint32)
        if (param->write.len >= 6 && param->write.value[0] == 0xFE) {
            uint8_t action = param->write.value[1];
            uint32_t secs = ((uint32_t)param->write.value[2] << 24) | ((uint32_t)param->write.value[3] << 16) |
                            ((uint32_t)param->write.value[4] << 8) | ((uint32_t)param->write.value[5]);

            if (action == 2) {
                // cancel
                if (schedule_timer) {
                    xTimerStop(schedule_timer, 0);
                    xTimerDelete(schedule_timer, 0);
                    schedule_timer = NULL;
                }
                scheduled_action = 0xFF;
                ESP_LOGI(TAG, "BLE: canceled scheduled action");
            } else {
                // Create or replace existing timer
                if (schedule_timer) {
                    xTimerStop(schedule_timer, 0);
                    xTimerDelete(schedule_timer, 0);
                    schedule_timer = NULL;
                }
                TickType_t ticks = pdMS_TO_TICKS((uint64_t)secs * 1000ULL);
                schedule_timer = xTimerCreate("sched", ticks, pdFALSE, NULL, schedule_timer_cb);
                if (schedule_timer) {
                    scheduled_action = action;
                    xTimerStart(schedule_timer, 0);
                    ESP_LOGI(TAG, "BLE: scheduled action %d in %u seconds", action, secs);
                } else {
                    ESP_LOGE(TAG, "BLE: failed to create schedule timer");
                }
            }
        } else if (param->write.len > 0) {
            uint8_t val = param->write.value[0];
            if (val > 100)
                val = 100;
            d_state.power_percent = (float)val;
            if (val > 1)
                d_state.last_nonzero_power = (float)val;

            if (param->write.len > 1)
            {
                uint8_t bias = param->write.value[1];
                if (bias > 100)
                    bias = 100;
                d_state.bias_percent = (float)bias;
                ESP_LOGI(TAG, "BLE set power: %d%% bias: %d%%", val, bias);
            }
            else
            {
                ESP_LOGI(TAG, "BLE set power: %d%%", val);
            }

            // Persist changes
            save_settings();
        }
        if (param->write.need_rsp)
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        }

        // Notify connected client of the updated values (power,bias)
        if (g_connected && g_char_handle != 0) {
            uint8_t notify_vals[2] = {(uint8_t)d_state.power_percent, (uint8_t)d_state.bias_percent};
            esp_ble_gatts_send_indicate(g_gatts_if, g_conn_id, g_char_handle, 2, notify_vals, false);
        }
        break;
    }

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "BLE connected");
        g_connected = true;
        g_gatts_if = gatts_if;
        g_conn_id = param->connect.conn_id;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "BLE disconnected, restarting advertising");
        g_connected = false;
        g_conn_id = 0;
        g_gatts_if = 0;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Start");

    setup_timer();
    setup_gpio();

    // Initialize NVS (required for BT)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Load persisted settings (power_percent, bias_percent)
    load_settings();

    // Initialize Bluetooth
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Register BLE callbacks
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_profile_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_APP_ID));

    while (1)
    {
        if (d_state.period_us > 0)
            d_state.frequency_hz = (float)TIMER_RESOLUTION_HZ / (float)d_state.period_us / 2.0f;

        ESP_LOGI(TAG, "Freq: %.2f Hz | Power: %.1f %% | Period: %lu us",
                 d_state.frequency_hz, d_state.power_percent, d_state.period_us);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}