#include <Arduino.h>
#include <cstdint>

/**
 * @brief class controles brushed dc motor with sensorless position control
 */
template <
    std::size_t N_MAX = 8,
    std::uint32_t PWM_FREQUENCY = 16000,
    std::uint32_t PWM_BIT_WIDTH = 12,
    std::uint32_t PWM_DUTY_MAX = 100000,
    std::uint32_t ADC_SAMPLE_RATE = 48000,
    std::uint32_t ADC_CONV_PER_PIN = 30,
    std::uint32_t ADC_BIT_WIDTH = 12,
    adc_attenuation_t ADC_ATTENUATION = ADC_11db,
    >
class BdcSensorlessPositionControl
{
public:
  enum
  {
    FORWARDS,
    BACKWARDS
  } direction_e;

protected:
  // array with all the ADCs
  static std::uint8_t adc_pins_[N_MAX] = {0};
  static std::uint8_t pwm_pins_[N_MAX] = {0};
  static std::uint8_t dir_pins_[N_MAX] = {0};
  static std::uint8_t no_of_instances_ = 0;
  // Result structure for ADC Continuous reading
  static adc_continuos_data_t *result = NULL;
  // Flag which will be set in ISR when conversion is done
  static volatile bool adc_coversion_done = false;

  // ISR Function that will be triggered when ADC conversion is done
  static inline void ARDUINO_ISR_ATTR adcComplete(void)
  {
    adc_coversion_done = true;
  }

  std::uint32_t adc_mean_;
  std::uint32_t pwm_duty_;
  std::uint8_t instance_index_;
  direction_e direction_;

public:
  BdcSensorlessPositionControl(
      const std::uint8_t pwm_pin,
      const std::uint8_t dir_pin,
      const std::uint8_t adc_pin) : adc_mean_(0),
                                    pwm_duty_(0),
                                    instance_index_(no_of_instances_),
                                    direction_(FORWARDS),
  {
    pwm_pins_[no_of_instances_] = pwm_pin;
    dir_pins_[no_of_instances_] = dir_pin;
    adc_pins_[no_of_instances_] = adc_pin;
    no_of_instances_++;
  }

  static void initADC(void)
  {
    // Optional for ESP32: Set the resolution to 9-12 bits (default is 12 bits)
    analogContinuousSetWidth(ADC_BIT_WIDTH);

    // Optional: Set different attenaution (default is ADC_11db)
    analogContinuousSetAtten(ADC_ATTENUATION);

    // Setup ADC Continuous with following input:
    // array of pins, count of the pins, how many conversions per pin in one cycle will happen, sampling frequency, callback function
    analogContinuous(adc_pins_, no_of_instances_, ADC_CONV_PER_PIN, ADC_SAMPLE_RATE, &adcComplete);

    // Start ADC Continuous conversions
    analogContinuousStart();
  }

  static void initPWM(void)
  {
    for (std::uint8_t i = 0; i < no_of_instances_; ++i)
    {
      // Setup timer and attach timer to a led pin
      ledcAttach(pwm_pins_[i], PWM_FREQUENCY, PWM_BIT_WIDTH);
      pinMode(dir_pins_[i], OUTPUT);
    }
  }

  void setDuty(const std::uint32_t duty, const direction_e dir)
  {
    constexpr std::uint32_t PWM_MAX = (1UL << PWM_BIT_WIDTH);
    std::uint32_t pwm_duty = (PWM_BIT_WIDTH / PWM_DUTY_MAX) * min(value, PWM_DUTY_MAX);

    if (dir == FORWARDS)
    {
      ledcWrite(pwm_pins_[instance_index_], pwm_duty);
      digitalWrite(dir_pins_[instance_index_], 0);
    }
    else
    {
      ledcWrite(pwm_pins_[instance_index_], PWM_MAX - pwm_duty);
      digitalWrite(dir_pins_[instance_index_], 1);
    }
  }
}
