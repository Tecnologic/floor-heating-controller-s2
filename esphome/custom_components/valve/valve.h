#pragma once
#include "esphome.h"

namespace esphome {
namespace valve {

class Valve : public Component {
 protected:
    static const uint32_t CONTROL_FREQ = 1000;
    static const uint32_t ADC_SAMPLE_FREQ = 80000;
    static const uint32_t ADC_READ_LEN = ADC_SAMPLE_FREQ / CONTROL_FREQ;

    
    void adc_init();
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
};


}  // namespace valve
}  // namespace esphome