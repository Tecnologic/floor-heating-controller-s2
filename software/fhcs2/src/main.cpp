#include <Arduino.h>
#include "bdc_control.h"

constexpr std::uint32_t NO_OF_MOTORS = 8UL;
using bdc = BdcSensorlessPositionControl<NO_OF_MOTORS>;
using bdc_array = std::array<bdc, NO_OF_MOTORS>;

bdc_array motors =
    {
        {.pwm_pin = 9, .dir_pin = 10, .adc_pin = 1},
        {.pwm_pin = 11, .dir_pin = 12, .adc_pin = 2},
        {.pwm_pin = 13, .dir_pin = 14, .adc_pin = 3},
        {.pwm_pin = 16, .dir_pin = 17, .adc_pin = 4},
        {.pwm_pin = 18, .dir_pin = 19, .adc_pin = 5},
        {.pwm_pin = 33, .dir_pin = 34, .adc_pin = 6},
        {.pwm_pin = 35, .dir_pin = 36, .adc_pin = 7},
        {.pwm_pin = 37, .dir_pin = 38, .adc_pin = 8},
};

void setup()
{
  Serial.begin(112500);
  delay(1000);
  bdc::init();
  ESP_LOGI(TAG, "Setup complete!");
}

void loop()
{
  std::uint32_t i = 0;
  for (auto &motor : motors)
  {
    i++;
    ESP_LOGI(TAG, "Motor %d Current %d uA", i, motor.getCurrent());
  }
}