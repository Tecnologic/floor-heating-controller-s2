#include "esphome/core/log.h"
#include "valve.h"
#include <cstdlib>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/adc.h>

namespace esphome {
namespace valve {

constexpr uint8_t GET_UNIT(const uint8_t x) {  return ((x>>3) & 0x1); };

static const char *TAG = "valve.component";
static const gpio_num_t LED_GPIO = static_cast<gpio_num_t>(15);

TickType_t last_tick = 0;
bool s_led_state = 0;

constexpr uint32_t TIMES = 256;
constexpr uint8_t ADC_RESULT_BYTE = 2;
constexpr uint8_t channel_num = 8;
static adc_channel_t channel[channel_num] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
                                             ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7};

static bool check_valid_data(const adc_digi_output_data_t *data)
{
    int unit = data->type2.unit;
    if (unit >= ADC_UNIT_2) {
        return false;
    }
    if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(unit)) {
        return false;
    }
    return true;
}


void Valve::adc_init()
{
   adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 1024,
        .conv_num_each_intr = 256,
        .adc1_chan_mask = 0xFF,
        .adc2_chan_mask = 0,
    };
    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    adc_digi_configuration_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        uint8_t unit = ADC_UNIT_1;
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_0;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg)); 

    adc_digi_start();
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
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[TIMES] = {0};
    memset(result, 0xcc, TIMES);

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

    ret = adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE)
    {
        if (ret == ESP_ERR_INVALID_STATE)
        {
            /**
             * @note 1
             * Issue:
             * As an example, we simply print the result out, which is super slow. Therefore the conversion is too
             * fast for the task to handle. In this condition, some conversion results lost.
             *
             * Reason:
             * When this error occurs, you will usually see the task watchdog timeout issue also.
             * Because the conversion is too fast, whereas the task calling `adc_digi_read_bytes` is slow.
             * So `adc_digi_read_bytes` will hardly block. Therefore Idle Task hardly has chance to run. In this
             * example, we add a `vTaskDelay(1)` below, to prevent the task watchdog timeout.
             *
             * Solution:
             * Either decrease the conversion speed, or increase the frequency you call `adc_digi_read_bytes`
             */
        }

        ESP_LOGI("TASK:", "ret is %x, ret_num is %d", ret, ret_num);
/*         for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE)
        {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
            if (check_valid_data(p))
            {
                ESP_LOGI(TAG, "Unit: %d,_Channel: %d, Value: %x", p->type2.unit + 1, p->type2.channel, p->type2.data);
            }
            else
            {
                ESP_LOGI(TAG, "Invalid data");
            }
        } */
        // See `note 1`
        vTaskDelay(10);
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        /**
         * ``ESP_ERR_TIMEOUT``: If ADC conversion is not finished until Timeout, you'll get this return error.
         * Here we set Timeout ``portMAX_DELAY``, so you'll never reach this branch.
         */
        ESP_LOGW(TAG, "No data, increase timeout or reduce conv_num_each_intr");
        vTaskDelay(1000);
    }
}

void Valve::dump_config(){
    ESP_LOGCONFIG(TAG, "Valve");
}


}  // namespace valve
}  // namespace esphome