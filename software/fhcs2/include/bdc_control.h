#ifndef BDC_SENSORLESS_POSITION_CONTROL_
#define BDC_SENSORLESS_POSITION_CONTROL_

#include <Arduino.h>
#include <cstdint>

/**
 * @brief class controles brushed dc motor with sensorless position control
 */
template <std::uint32_t N_MAX_ = 8>
class BdcSensorlessPositionControl
{
public:
  // PWM frequency
  static constexpr std::uint32_t PWM_FREQUENCY = 16000;
  // PWM bit width
  static constexpr std::uint32_t PWM_BIT_WIDTH = 12;
  // PWM supply voltage in mV
  static constexpr std::uint32_t PWM_VOLTAGE = 5000; // supply voltage in mv
  // ADC sample rate
  static constexpr std::uint32_t ADC_SAMPLE_RATE = 80000;
  // ADC conversions per pin
  static constexpr std::uint32_t ADC_CONV_PER_PIN = 30;
  // ADC Resolution in bits
  static constexpr std::uint32_t ADC_BIT_WIDTH = 12;
  // ADC input attenuation
  static constexpr adc_attenuation_t ADC_ATTENUATION = ADC_11db;
  // Conversion factor from mV to uA
  static constexpr std::uint32_t MV_TO_UA = 70;
  // Number of offset samples
  static constexpr std::uint32_t OFFSET_SAMPLES = 1000;
  // TASK notification index
  static constexpr std::uint32_t TASK_NOTIFICATION_INDEX = 1;

protected:
  // array with all the class instances
  static std::array<BdcSensorlessPositionControl *, N_MAX_> instances_;
  // number of elements in instances_ array
  static std::uint32_t no_of_instances_;
  // Result structure for ADC Continuous reading
  static adc_continuos_data_t *adc_result_;
  // Flag which will be set in ISR when conversion is done
  static volatile bool adc_coversion_done_;
  // handle to the control task
  static TaskHandle_t taskHandle_;

  // ISR Function that will be triggered when ADC conversion is done
  static inline void ARDUINO_ISR_ATTR adcComplete(void)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Notify the task that the transmission is complete. */
    vTaskNotifyGiveFromISR(taskHandle_,
                           &xHigherPriorityTaskWoken);
    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a
    context switch should be performed to ensure the interrupt
    returns directly to the highest priority task.  The macro used
    for this purpose is dependent on the port in use and may be
    called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  static void taskControl(void *parameter)
  {
    while (1)
    {
      uint32_t notification_value = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(5000));

      if (notification_value)
      {
        // ADC Conversion was done
        // Read data from ADC
        if (analogContinuousRead(&adc_result_, 0))
        {
          for (int i = 0; i < no_of_instances_; i++)
          {
            if (instances_[i]->offset_samples_ < OFFSET_SAMPLES)
            {
              instances_[i]->offset_current_ += adc_result_[i].avg_read_mvolts * MV_TO_UA;
              instances_[i]->offset_samples_++;
            }
            else if (instances_[i]->offset_samples_ == OFFSET_SAMPLES)
            {
              instances_[i]->offset_current_ /= instances_[i]->offset_samples_;
              instances_[i]->offset_samples_++;
              Serial.printf("Motor %d current offset %d uA\n", i, instances_[i]->offset_current_);
            }
            else
            {
              instances_[i]->actual_current_ = adc_result_[i].avg_read_mvolts * MV_TO_UA - instances_[i]->offset_current_;
            }
          }
        }
        else
        {
          Serial.println("Error occured during reading adc data.");
        }
      }
      else
      {
        // Timeout
        Serial.println("ADC Timeout");
      }
    }
  }

  // index of the instance
  std::uint32_t index_of_instance_;
  // current measurement adc input pin
  std::uint8_t adc_pin_;
  // pwm output for the motor
  std::uint8_t pwm_pin_;
  // 2nd output to change direction of rotation of the motor
  std::uint8_t dir_pin_;
  // current measurement in uA
  std::int32_t actual_current_;
  // set current in uA
  std::int32_t set_current_;
  // current offset in uA
  std::int32_t offset_current_;
  // output voltage in mV
  std::uint32_t output_voltage_;
  // actual position
  std::int32_t actual_position_;
  // set position
  std::int32_t set_position_;
  // offset samples
  std::uint32_t offset_samples_;

public:
  /**
   * @brief main constuctor
   */
  BdcSensorlessPositionControl(
      const std::uint8_t pwm_pin = 9,
      const std::uint8_t dir_pin = 10,
      const std::uint8_t adc_pin = 1) : index_of_instance_(no_of_instances_),
                                        adc_pin_(adc_pin),
                                        pwm_pin_(pwm_pin),
                                        dir_pin_(dir_pin),
                                        actual_current_(0),
                                        set_current_(0),
                                        offset_current_(0),
                                        output_voltage_(0),
                                        actual_position_(0),
                                        set_position_(0),
                                        offset_samples_(0)
  {
    instances_[index_of_instance_] = this;
    no_of_instances_++;
  }

  /**
   * @brief static initialization
   */
  static void init(void)
  {
    initADC();
    initPWM();

    xTaskCreate(
        taskControl,   /* Task function. */
        "taskControl", /* String with name of task. */
        10000,         /* Stack size in bytes. */
        NULL,          /* Parameter passed as input of the task */
        1,             /* Priority of the task. */
        &taskHandle_); /* Task handle. */
  }

  /**
   * @brief setup adc conversion
   */
  static void initADC(void)
  {
    std::array<std::uint8_t, N_MAX_> adc_pins = {0};

    for (std::uint32_t i = 0; i < no_of_instances_; ++i)
    {
      adc_pins[i] = instances_[i]->adc_pin_;
    }

    // Optional for ESP32: Set the resolution to 9-12 bits (default is 12 bits)
    analogContinuousSetWidth(ADC_BIT_WIDTH);

    // Optional: Set different attenaution (default is ADC_11db)
    analogContinuousSetAtten(ADC_ATTENUATION);

    // Setup ADC Continuous with following input:
    // array of pins, count of the pins, how many conversions per pin in one cycle will happen, sampling frequency, callback function
    analogContinuous(adc_pins.data(), no_of_instances_, ADC_CONV_PER_PIN, ADC_SAMPLE_RATE, &adcComplete);

    // Start ADC Continuous conversions
    analogContinuousStart();
  }

  /**
   * @brief setup all PWM channels
   */
  static void initPWM(void)
  {
    Serial.println("PWM INIT");
    for (std::uint8_t i = 0; i < no_of_instances_; ++i)
    {
      Serial.print("Instance ");
      Serial.print(i);
      instances_[i]->initPins();
    }
  }

  /**
   * @brief init the PWM Pins
   */
  void initPins(void)
  {
    Serial.print(" PWM Pin ");
    Serial.print(pwm_pin_);
    // Setup timer and attach timer to a led pin
    // ledcAttach(instances_[i]->pwm_pin_, PWM_FREQUENCY, PWM_BIT_WIDTH);
    Serial.print(" Dir Pin ");
    Serial.println(dir_pin_);
    pinMode(dir_pin_, OUTPUT);
  }

  /**
   * @brief set the output voltage for the motor
   */
  void setVoltage(const std::int32_t voltage)
  {
    constexpr std::int32_t PWM_MAX = (1UL << PWM_BIT_WIDTH);
    std::int32_t duty = (PWM_MAX / PWM_VOLTAGE) * voltage;
    std::uint8_t dir = 0;

    duty = min(duty, PWM_MAX);

    if (duty < 0)
    {
      duty = PWM_MAX - max(duty, -PWM_MAX);
      dir = 1;
    }

    ledcWrite(pwm_pin_, duty);
    digitalWrite(dir_pin_, dir);
  }

  /**
   * @brief get the acutal current of the motor
   */
  std::int32_t getCurrent(void)
  {
    return actual_current_;
  }
};

// array with all the class instances
template <std::uint32_t N_MAX_>
std::array<BdcSensorlessPositionControl<N_MAX_> *, N_MAX_> BdcSensorlessPositionControl<N_MAX_>::instances_ = {nullptr};
// number of elements in instances_ array
template <std::uint32_t N_MAX_>
std::uint32_t BdcSensorlessPositionControl<N_MAX_>::no_of_instances_ = 0;
// Result structure for ADC Continuous reading
template <std::uint32_t N_MAX_>
adc_continuos_data_t *BdcSensorlessPositionControl<N_MAX_>::adc_result_ = nullptr;
// Flag which will be set in ISR when conversion is done
template <std::uint32_t N_MAX_>
volatile bool BdcSensorlessPositionControl<N_MAX_>::adc_coversion_done_ = false;
// handle to the control task
template <std::uint32_t N_MAX_>
TaskHandle_t BdcSensorlessPositionControl<N_MAX_>::taskHandle_ = nullptr;

#endif /* BDC_SENSORLESS_POSITION_CONTROL_ */
