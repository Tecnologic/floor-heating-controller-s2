#include <Arduino.h>
#include "bdc_control.h"

std::array<BdcSensorlessPositionControl, 8> motors =
    {
        {.pwm_pin = 9, .dir_pin = 10, adc_pin = 1},
        {.pwm_pin = 11, .dir_pin = 12, adc_pin = 2},
        {.pwm_pin = 13, .dir_pin = 14, adc_pin = 3},
        {.pwm_pin = 16, .dir_pin = 17, adc_pin = 4},
        {.pwm_pin = 18, .dir_pin = 19, adc_pin = 5},
        {.pwm_pin = 33, .dir_pin = 34, adc_pin = 6},
        {.pwm_pin = 35, .dir_pin = 36, adc_pin = 7},
        {.pwm_pin = 37, .dir_pin = 38, adc_pin = 8},
};

void setup()
{
  Serial.begin(112500);
  delay(1000);
  BdcSensorlessPositionControl::init();
}

void loop()
{
  // Check if conversion is done and try to read data
  if (adc_coversion_done == true)
  {
    // Set ISR flag back to false
    adc_coversion_done = false;
    // Read data from ADC
    if (analogContinuousRead(&result, 0))
    {
      for (int i = 0; i < adc_pins_count; i++)
      {
        Serial.printf("\nADC PIN %d data:", result[i].pin);
        Serial.printf("  Avg raw value = %d", result[i].avg_read_raw);
        Serial.printf("  Avg millivolts value = %d", result[i].avg_read_mvolts);
      }

      // Delay for better readability of ADC data
      delay(1000);
    }
    else
    {
      Serial.println("Error occured during reading data. Set Core Debug Level to error or lower for more informations.");
    }
  }
}