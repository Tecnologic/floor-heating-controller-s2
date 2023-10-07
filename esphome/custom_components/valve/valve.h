#pragma once
#include "esphome.h"

namespace esphome {
namespace valve {

class Valve : public Component {
 protected:
    static const uint32_t CONTROL_FREQ = 1000;
    static const uint32_t ADC_SAMPLE_FREQ = 80000;
    static const uint32_t ADC_READ_LEN = ADC_SAMPLE_FREQ / CONTROL_FREQ;

    adc_channel_t adc_chan;
    adc_continuous_handle_t adc_handle;
    adc_continuous_handle_cfg_t adc_config;
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX];

    void adc_init();
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
};


}  // namespace valve
}  // namespace esphome