#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "rom/ets_sys.h" // For ets_delay_us
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include <string.h>

#define PIN_ZCD_INPUT GPIO_NUM_23   // Input from Zero Cross Detector
#define PIN_TRIAC_GATE GPIO_NUM_22  // Output to TRIAC Gate
#define TRIAC_PULSE_US 500          // Duration of the trigger pulse in microseconds
#define MAINS_FREQ_DEFAULT 60       // Fallback frequency if detection fails
#define TIMER_RESOLUTION_HZ 1000000 // 1MHz resolution (1 tick = 1us)

static const char *TAG = "DIMMER";

typedef struct
{
    uint32_t period_us;     // Calculated period of half-cycle
    float frequency_hz;     // Calculated Frequency
    float power_percent;    // Target Power (0.0 to 100.0)
    float bias_percent;     // Hardware offset bias (0.0 to 100.0)
    uint64_t last_zcd_time; // Timestamp of last ZCD interrupt
} dimmer_state_t;

volatile dimmer_state_t d_state = {
    .period_us = 8333,
    .frequency_hz = MAINS_FREQ_DEFAULT,
    .power_percent = 0.0, // Start off
    .bias_percent = 1.0,  // Example: 5% offset
    .last_zcd_time = 0};

gptimer_handle_t gptimer = NULL;
static QueueHandle_t zcd_queue = NULL;

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

// Timer Alarm ISR
static bool IRAM_ATTR triac_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    gpio_set_level(PIN_TRIAC_GATE, 1);
    ets_delay_us(TRIAC_PULSE_US);
    gpio_set_level(PIN_TRIAC_GATE, 0);

    gptimer_stop(timer);

    return false; // No task yield needed
}

// GPIO ISR
static void IRAM_ATTR zcd_isr_handler(void *arg)
{
    uint64_t current_time = esp_timer_get_time();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (zcd_queue)
    {
        xQueueSendFromISR(zcd_queue, &current_time, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR();
    }
}

// Process ZCD events and arm timer / fire triac as needed
static void zcd_task(void *arg)
{
    uint64_t timestamp = 0;
    uint64_t last_time = 0;

    for (;;)
    {
        if (xQueueReceive(zcd_queue, &timestamp, portMAX_DELAY) != pdTRUE)
            continue;

        uint32_t diff = (uint32_t)(timestamp - last_time);
        last_time = timestamp;
        d_state.last_zcd_time = timestamp;

        if (diff > 2000 && diff < 15000)
        {
            d_state.period_us = diff;
        }

        float target_power = d_state.power_percent;
        if (target_power <= 1.0f)
        {
            gptimer_stop(gptimer);
            continue;
        }

        if (target_power >= 99.0f)
        {
            gpio_set_level(PIN_TRIAC_GATE, 1);
            ets_delay_us(TRIAC_PULSE_US);
            gpio_set_level(PIN_TRIAC_GATE, 0);
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
        .intr_type = GPIO_INTR_POSEDGE // Trigger on Rising Edge (Start of logic ZCD)
    };
    gpio_config(&io_conf_in);

    // Install ISR Service
    gpio_install_isr_service(0);
    // Create queue and zcd processing task
    zcd_queue = xQueueCreate(10, sizeof(uint64_t));
    if (zcd_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create ZCD queue");
    }
    else
    {
        xTaskCreate(zcd_task, "zcd_task", 3072, NULL, configMAX_PRIORITIES - 1, NULL);
    }
    gpio_isr_handler_add(PIN_ZCD_INPUT, zcd_isr_handler, NULL);
}

void setup_timer()
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ, // 1us tick
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = triac_timer_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
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
                               ESP_GATT_CHAR_PROP_BIT_WRITE,
                               NULL, NULL);

        esp_ble_gatts_start_service(param->create.service_handle);
        break;
    }

    case ESP_GATTS_WRITE_EVT:
    {
        if (param->write.len > 0)
        {
            uint8_t val = param->write.value[0];
            if (val > 100)
                val = 100;
            d_state.power_percent = (float)val;
            ESP_LOGI(TAG, "BLE set power: %d%%", val);
        }

        if (param->write.need_rsp)
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        }
        break;
    }

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "BLE connected");
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "BLE disconnected, restarting advertising");
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
            d_state.frequency_hz = 1000000.0f / (float)d_state.period_us / 2.0f;

        ESP_LOGI(TAG, "Freq: %.2f Hz | Power: %.1f %% | Period: %lu us",
                 d_state.frequency_hz, d_state.power_percent, d_state.period_us);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}