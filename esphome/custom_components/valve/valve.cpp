#include "esphome/core/log.h"
#include "valve.h"
#include <cstdlib>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/adc.h>

namespace esphome {
namespace valve {

static const char *TAG = "valve.component";
static const gpio_num_t LED_GPIO = static_cast<gpio_num_t>(15);

TickType_t last_tick = 0;
bool s_led_state = 0;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

void valve::adc_init()
{
    adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = ADC_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = ADC_SAMPLE_FREQ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = SOC_ADC_DIGI_MAX_BITWIDTH,
    };

    std::memset(&dig_cfg, 0, sizeof(dig_cfg));
    dig_cfg.pattern_num = channel_num;
    adc_pattern[0].atten = ADC_ATTEN_DB_0;
    adc_pattern[0].channel = channel & 0x7;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    ESP_LOGI(TAG, "adc_pattern[0].atten is :%"PRIx8, adc_pattern[0].atten);
    ESP_LOGI(TAG, "adc_pattern[0].channel is :%"PRIx8, adc_pattern[0].channel);
    ESP_LOGI(TAG, "adc_pattern[0].unit is :%"PRIx8, adc_pattern[0].unit);
    
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
}


void Valve::setup() {
    ESP_LOGI(TAG, "Valve Setup GPIO %d as output", LED_GPIO);

    gpio_reset_pin(LED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    last_tick = xTaskGetTickCount();

    adc_init();
}

void Valve::loop() {
    
    TickType_t current_tick = xTaskGetTickCount();
    TickType_t diff = pdMS_TO_TICKS(1000);
    if((current_tick - last_tick) > diff)
    {
        current_tick = xTaskGetTickCount();
        last_tick = current_tick;
        /* Set the GPIO level according to the state (LOW or HIGH)*/
        ESP_LOGI(TAG, "Turning the LED %s! at Tick %d, Last %d, Diff %d\n", s_led_state == true ? "ON" : "OFF", current_tick, last_tick, diff);
        gpio_set_level(LED_GPIO, s_led_state);
        /* Toggle the LED state */
        s_led_state = !s_led_state;
    }
}

void Valve::dump_config(){
    ESP_LOGCONFIG(TAG, "Valve");
}


}  // namespace valve
}  // namespace esphome