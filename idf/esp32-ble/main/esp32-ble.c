#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "driver/gpio.h"

#define TAG "BLE_LED"

#define LED_GPIO GPIO_NUM_23
#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

// UUIDs - Must match the Web App
#define SERVICE_UUID        {0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f, 0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f}
#define CHAR_UUID           {0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7, 0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe}

static uint16_t led_handle_table[4];

// GPIO Initialization
void setup_gpio() {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0); // Start OFF
}

// Advertising Data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, 
    .max_interval = 0x0010, 
    .appearance = 0x00,
    .manufacturer_len = 0, 
    .p_manufacturer_data =  NULL, 
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = (uint8_t[]){0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f, 0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f}, // Service UUID reversed
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    // .filter_policy      = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GAP Callback (Advertising)
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    default:
        break;
    }
}

// GATTS Callback (Read/Write Events)
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        esp_ble_gap_set_device_name("ESP32_LED_CONTROL");
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

    case ESP_GATTS_CREATE_EVT:
        // Add Characteristic
        ; // Empty statement for variable declaration
        uint8_t char_uuid128[16] = CHAR_UUID;
        esp_bt_uuid_t char_uuid;
        char_uuid.len = ESP_UUID_LEN_128;
        memcpy(char_uuid.uuid.uuid128, char_uuid128, 16);

        esp_ble_gatts_add_char(param->create.service_handle, &char_uuid,
                               ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_WRITE, 
                               NULL, NULL);
        
        // Start Service
        esp_ble_gatts_start_service(param->create.service_handle);
        break;

    case ESP_GATTS_WRITE_EVT:
        // HANDLE DATA FROM WEB APP
        if (param->write.len > 0) {
            uint8_t val = param->write.value[0];
            ESP_LOGI(TAG, "Received data: %d", val);
            
            if (val == 1) {
                gpio_set_level(LED_GPIO, 1);
                ESP_LOGI(TAG, "LED ON");
            } else {
                gpio_set_level(LED_GPIO, 0);
                ESP_LOGI(TAG, "LED OFF");
            }
        }
        
        // Send response if needed (write with response)
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        }
        break;
        
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "Web App Connected");
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Web App Disconnected, Restarting Advertising");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}

void app_main(void) {
    // 1. Setup GPIO
    setup_gpio();

    // 2. Initialize NVS (Required for WiFi/BT)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 3. Initialize Bluetooth Controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // 4. Initialize Bluedroid Stack
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // 5. Register Callbacks
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_profile_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_APP_ID));
    
    ESP_LOGI(TAG, "ESP32 BLE System Ready");
}