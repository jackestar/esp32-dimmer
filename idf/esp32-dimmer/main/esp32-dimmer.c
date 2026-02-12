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

// ================= CONFIGURATION =================
#define PIN_ZCD_INPUT       GPIO_NUM_23   // Input from Zero Cross Detector
#define PIN_TRIAC_GATE      GPIO_NUM_22   // Output to TRIAC Gate
#define TRIAC_PULSE_US      500           // Duration of the trigger pulse in microseconds
#define MAINS_FREQ_DEFAULT  60           // Fallback frequency if detection fails
#define TIMER_RESOLUTION_HZ 1000000      // 1MHz resolution (1 tick = 1us)

// ================= GLOBALS =================
static const char *TAG = "DIMMER";

// Volatile variables shared between ISR and Main Task
typedef struct {
    uint32_t period_us;         // Calculated period of half-cycle (e.g., ~8333us for 60Hz)
    float frequency_hz;         // Calculated Frequency
    float power_percent;        // Target Power (0.0 to 100.0)
    float bias_percent;         // Hardware offset bias (0.0 to 100.0)
    uint64_t last_zcd_time;     // Timestamp of last ZCD interrupt
} dimmer_state_t;

volatile dimmer_state_t d_state = {
    .period_us = 8333,
    .frequency_hz = MAINS_FREQ_DEFAULT,
    .power_percent = 0.0,      // Start off
    .bias_percent = 5.0,       // Example: 5% offset
    .last_zcd_time = 0
};

gptimer_handle_t gptimer = NULL;
static QueueHandle_t zcd_queue = NULL;

// ================= INTERRUPT HANDLERS =================

// 1. Timer Alarm ISR: This fires when it's time to trigger the TRIAC
static bool IRAM_ATTR triac_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    // Generate a short pulse to latch the TRIAC
    // Note: ets_delay_us is a busy-wait, but 20us is negligible for ESP32
    gpio_set_level(PIN_TRIAC_GATE, 1);
    ets_delay_us(TRIAC_PULSE_US);
    gpio_set_level(PIN_TRIAC_GATE, 0);

    // Stop the timer so it doesn't fire again until next ZCD re-arms it
    gptimer_stop(timer);
    
    return false; // No task yield needed
}

// 2. GPIO ISR: This fires at Zero Crossing
// ISR is minimal: capture timestamp and notify a task to handle heavy work
static void IRAM_ATTR zcd_isr_handler(void* arg)
{
    uint64_t current_time = esp_timer_get_time();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (zcd_queue) {
        xQueueSendFromISR(zcd_queue, &current_time, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR();
    }
}

// Background task: process ZCD events and arm timer / fire triac as needed
static void zcd_task(void *arg)
{
    uint64_t timestamp = 0;
    uint64_t last_time = 0;

    for (;;) {
        if (xQueueReceive(zcd_queue, &timestamp, portMAX_DELAY) != pdTRUE) continue;

        uint32_t diff = (uint32_t)(timestamp - last_time);
        last_time = timestamp;
        d_state.last_zcd_time = timestamp;

        if (diff > 2000 && diff < 15000) {
            d_state.period_us = diff;
        }

        float target_power = d_state.power_percent;
        if (target_power <= 1.0f) {
            gptimer_stop(gptimer);
            continue;
        }

        if (target_power >= 99.0f) {
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

        if (fire_delay < 100) fire_delay = 100;
        if (fire_delay > d_state.period_us - 200) {
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

// ================= SETUP FUNCTIONS =================

void setup_gpio()
{
    // Setup Output (Triac Gate)
    gpio_config_t io_conf_out = {
        .pin_bit_mask = (1ULL << PIN_TRIAC_GATE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_out);
    gpio_set_level(PIN_TRIAC_GATE, 0);

    // Setup Input (ZCD)
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
    if (zcd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create ZCD queue");
    } else {
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

void app_main(void)
{
    ESP_LOGI(TAG, "Start");
    
    setup_timer();
    setup_gpio();

    // Simulation loop: Ramp power up and down
    float test_power = 10.0;
    int direction = 1;

    while (1) {
        // Update global frequency calculation for display
        // Note: period_us is updated in ISR. ZCD happens 2x per mains cycle.
        // Freq = 1,000,000 / period_us / 2
        if (d_state.period_us > 0) {
            d_state.frequency_hz = 1000000.0f / (float)d_state.period_us / 2.0f;
        }

        // Apply Power
        d_state.power_percent = test_power;

        ESP_LOGI(TAG, "Freq: %.2f Hz | Power: %.1f %% | Period: %lu us", 
                 d_state.frequency_hz, d_state.power_percent, d_state.period_us);

        // Ramp Logic
        test_power += (5.0 * direction);
        if (test_power >= 95.0) direction = -1;
        if (test_power <= 5.0) direction = 1;

        vTaskDelay(pdMS_TO_TICKS(500)); // Update every 500ms
    }
}