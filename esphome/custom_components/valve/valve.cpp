#include "valve.h"
namespace esphome {
namespace valve {

const char* TAG = "valve.component";

bool Valve::check_valid_data(const adc_digi_output_data_t *data)
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

   channel[0] = ADC_CHANNEL_0;

   adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = MAX_BUFFER_SIZE,
        .conv_num_each_intr = TIMES,
        .adc1_chan_mask = 0x01,
        .adc2_chan_mask = 0,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(adc_digi_initialize(&adc_dma_config));

    adc_digi_configuration_t dig_cfg = {
        .sample_freq_hz = ADC_SAMPLE_FREQ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
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
    ESP_ERROR_CHECK_WITHOUT_ABORT(adc_digi_controller_configure(&dig_cfg)); 

    adc_digi_start();
}


void Valve::setup() {
    ESP_LOGI(TAG, "Valve Setup GPIO %d as output", LED_GPIO);

    gpio_reset_pin(LED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    led_tick = xTaskGetTickCount();
    adc_tick = xTaskGetTickCount();

    adc_init();
}

void Valve::loop() {
    memset(result, 0xcc, TIMES);

    current_tick = xTaskGetTickCount();
    diff_tick = pdMS_TO_TICKS(1000);
    if((current_tick - led_tick) > diff_tick)
    {
        led_tick = current_tick;
        /* Set the GPIO level according to the state (LOW or HIGH)*/
        //ESP_LOGI(TAG, "Turning the LED %s! at Tick %d, Last %d, Diff %d\n", led_state == true ? "ON" : "OFF", current_tick, led_tick, diff_tick);
        gpio_set_level(LED_GPIO, led_state);
        /* Toggle the LED state */
        led_state = !led_state;

        uint32_t sum = 0;
        uint32_t cnt = 0;
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[0];
        for (uint32_t i = 0; i < ret_num; i += ADC_RESULT_BYTE)
        {
            sum += p->type1.data;
            cnt++;
            p++;
        }
        uint32_t mean = sum / cnt;
        p = (adc_digi_output_data_t *)&result[0];
        ESP_LOGI(TAG, "Unit: %d,_Channel: %d, Value: %d, Count: %d", p->type2.unit + 1, p->type2.channel, mean, cnt);
    }

    diff_tick = pdMS_TO_TICKS(1);
    if((current_tick - adc_tick) > diff_tick)
    {
        adc_tick = current_tick;
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
                ESP_LOGW(TAG, "ADC Invalid State, Either decrease the conversion speed, or increase the frequency you call `adc_digi_read_bytes'");
            }
        }
        else if (ret == ESP_ERR_TIMEOUT)
        {
            /**
             * ``ESP_ERR_TIMEOUT``: If ADC conversion is not finished until Timeout, you'll get this return error.
             * Here we set Timeout ``portMAX_DELAY``, so you'll never reach this branch.
             */
            ESP_LOGW(TAG, "No data, increase timeout or reduce conv_num_each_intr");
        }
    }
}

void Valve::dump_config(){
    ESP_LOGCONFIG(TAG, "Valve");
}


}  // namespace valve
}  // namespace esphome