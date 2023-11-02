#include "valve.h"

namespace esphome
{
    namespace valve
    {
        const char *TAG = "valve.component";

        /// @brief flag to prevent multiple hardware inits
        bool Valve::initialized = false;
        /// @brief array of all adc channels used by the valve instances
        adc_channel_t Valve::channels_[9];
        /// @brief number of adc channels used by the valve instances
        uint8_t Valve::num_channels_ = 0;

        uint32_t Valve::adc_result = 0;

        TaskHandle_t Valve::task_handle_;

        /**
         * @brief adc callback called when conversion of frame is done. The function just sends a signal to the task.
         */
        bool IRAM_ATTR Valve::adc_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
        {
            BaseType_t mustYield = pdFALSE;
            esp_err_t ret;
            uint32_t ret_num = 0;
            uint8_t result[EXAMPLE_READ_LEN] = {0};
            memset(result, 0xcc, EXAMPLE_READ_LEN);

            // Notify that ADC continuous driver has done enough number of conversions
            vTaskNotifyGiveFromISR(task_handle_, &mustYield);

            return (mustYield == pdTRUE);
        }

        /**
         * @brief initialize the adc unit with all channels of the different instances
         */
        void Valve::adc_init(void)
        {
            adc_continuous_handle_t handle = NULL;

            adc_continuous_handle_cfg_t adc_config = {
                .max_store_buf_size = ADC_BUFFER_LEN,
                .conv_frame_size = ADC_READ_LEN,
            };
            ESP_ERROR_CHECK_WITHOUT_ABORT(adc_continuous_new_handle(&adc_config, &handle));

            adc_continuous_config_t dig_cfg = {
                .sample_freq_hz = ADC_SAMPLE_FREQ,
                .conv_mode = ADC_CONV_SINGLE_UNIT_1,
                .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
            };

            adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
            dig_cfg.pattern_num = channel_num;
            for (int i = 0; i < channel_num; i++)
            {
                adc_pattern[i].atten = ADC_ATTEN_DB_11;
                adc_pattern[i].channel = channel[i] & 0x7;
                adc_pattern[i].unit = ADC_UNIT_1;
                adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

                ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
                ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
                ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
            }
            dig_cfg.adc_pattern = adc_pattern;
            ESP_ERROR_CHECK_WITHOUT_ABORT(adc_continuous_config(handle, &dig_cfg));

            adc_continuous_evt_cbs_t cbs = {
                .on_conv_done = adc_conv_done_cb,
            };
            ESP_ERROR_CHECK_WITHOUT_ABORT(adc_continuous_register_event_callbacks(handle, &cbs, &adc_result));
            ESP_ERROR_CHECK_WITHOUT_ABORT(adc_continuous_start(handle));
        }

        /**
         * @brief init the hardware for all instances at once
         */
        void Valve::init(void)
        {
            task_handle_ = xTaskGetCurrentTaskHandle();

            adc_init();
        }

        void Valve::setup()
        {
            channels_[num_channels_] = ADC_CHANNEL_1;
            num_channels_++;

            if (!initialized)
            {
                init();

                initialized = true;
            }
        }

        void Valve::loop()
        {
        }

        void Valve::dump_config()
        {
            ESP_LOGCONFIG(TAG, "Valve");
        }

    } // namespace valve
} // namespace esphome
