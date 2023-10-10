#pragma once
#include <cstdlib>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/adc.h>
#include "esphome.h"
#include "esphome/core/log.h"

namespace esphome {
namespace valve {

class Valve : public Component {
 protected:
   static constexpr uint32_t  CONTROL_FREQ = 1000;
   static constexpr uint32_t  ADC_SAMPLE_FREQ = 8000;
   static constexpr uint32_t  ADC_READ_LEN = ADC_SAMPLE_FREQ / CONTROL_FREQ;
   static constexpr uint8_t   GET_UNIT(const uint8_t x) {  return ((x>>3) & 0x1); };
   static constexpr gpio_num_t LED_GPIO = static_cast<gpio_num_t>(15);
   static constexpr uint32_t  MAX_BUFFER_SIZE = 1024;
   static constexpr uint32_t  TIMES = 256;
   static constexpr uint8_t   ADC_RESULT_BYTE = 2;
   static constexpr uint8_t   channel_num = 1;

   esp_err_t ret;
   uint32_t ret_num;
   uint8_t result[TIMES];
   TickType_t current_tick;
   TickType_t diff_tick;
   TickType_t led_tick;
   TickType_t adc_tick;
   bool led_state;
   adc_channel_t channel[channel_num];


   bool check_valid_data(const adc_digi_output_data_t *data);
   void adc_init();

 public:

   void setup() override;
   void loop() override;
   void dump_config() override;

};


}  // namespace valve
}  // namespace esphome