#pragma once
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"

namespace esphome
{
  namespace valve
  {

    class Valve : public Component
    {
    protected:
      static constexpr uint32_t CONTROL_FREQ = 10;
      static constexpr uint32_t ADC_SAMPLE_FREQ = 400;
      static constexpr uint32_t ADC_READ_LEN = ADC_SAMPLE_FREQ / CONTROL_FREQ;
      static constexpr uint32_t ADC_BUFFER_LEN = ADC_READ_LEN * 10;
      static adc_channel_t channels_[9];
      static uint8_t num_channels_;
      static TaskHandle_t task_handle_;
      static bool initialized;
      static uint32_t adc_result;

      static bool IRAM_ATTR Valve::adc_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
      static void adc_init(void);

      static void init(void);

      esp_adc_cal_characteristics_t cal_characteristics_[SOC_ADC_ATTEN_NUM] = {};

    public:
      void setup() override;
      void loop() override;
      void dump_config() override;
    };

  } // namespace valve
} // namespace esphome