#pragma once
#include <cstdlib>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/adc.h>
#include "esphome.h"
#include "esphome/core/log.h"

namespace esphome
{
  namespace valve
  {

    class Valve : public Component
    {
    protected:
      static constexpr uint32_t CONTROL_FREQ = 1000;
      static constexpr uint32_t ADC_SAMPLE_FREQ = 8000;
      static constexpr uint32_t ADC_READ_LEN = ADC_SAMPLE_FREQ / CONTROL_FREQ;

    public:
      void setup() override;
      void loop() override;
      void dump_config() override;
    };

  } // namespace valve
} // namespace esphome