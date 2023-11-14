#include <Arduino.h>
#include <cstdint>

/**
 * @brief class controles brushed dc motor with sensorless position control
 */
template <std::uint32_t N_MAX_ = 8,
          std::uint32_t PWM_FREQUENCY = 16000,
          std::uint32_t PWM_BIT_WIDTH = 12,
          std::uint32_t PWM_VOLTAGE = 5000, // supply voltage in mv
          std::uint32_t ADC_SAMPLE_RATE = 80000,
          std::uint32_t ADC_CONV_PER_PIN = 30,
          std::uint32_t ADC_BIT_WIDTH = 12,
          adc_attenuation_t ADC_ATTENUATION = ADC_11db,
          std::uint32_t TASK_NOTIFICATION_INDEX = 1>
class BdcSensorlessPositionControl
{
public:
protected:
  // Logging Tag
  static constexpr char *TAG = "BDC";
  // array with all the class instances
  static std::array<BdcSensorlessPositionControl *, N_MAX_> instances_ = {0};
  // number of elements in instances_ array
  static std::uint32_t no_of_instances_ = 0;
  // Result structure for ADC Continuous reading
  static adc_continuos_data_t *adc_result_ = NULL;
  // Flag which will be set in ISR when conversion is done
  static volatile bool adc_coversion_done_ = false;
  // handle to the control task
  static TaskHandle_t taskHandle_ = NULL;

  // ISR Function that will be triggered when ADC conversion is done
  static inline void ARDUINO_ISR_ATTR adcComplete(void)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Notify the task that the transmission is complete. */
    vTaskNotifyGiveIndexedFromISR(taskHandle_,
                                  TASK_NOTIFICATION_INDEX,
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
      uint32_t notification_value = xTaskNotifyTakeIndexed(TASK_NOTIFICATION_INDEX, pdTRUE, pdMS_TO_TICKS(1000));

      if (notification_value == 1)
      {
        // ADC Conversion was done
      }
      else
      {
        // Timeout
        ESP_LOGE(TAG, "ADC Timeout");
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
  // output voltage in mV
  std::uint32_t output_voltage_;

public:
  /**
   * @brief main constuctor
   */
  BdcSensorlessPositionControl(
      const std::uint8_t pwm_pin = 8,
      const std::uint8_t dir_pin = 9,
      const std::uint8_t adc_pin = 1) : index_of_instance_(no_of_instances_),
                                        adc_pin_(adc_pin),
                                        pwm_pin_(pwm_pin),
                                        dir_pin_(dir_pin),
                                        actual_current_(0),
                                        output_voltage_(0),
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
      adc_pins[i] = instances_[i].adc_pin_;
    }

    // Optional for ESP32: Set the resolution to 9-12 bits (default is 12 bits)
    analogContinuousSetWidth(ADC_BIT_WIDTH);

    // Optional: Set different attenaution (default is ADC_11db)
    analogContinuousSetAtten(ADC_ATTENUATION);

    // Setup ADC Continuous with following input:
    // array of pins, count of the pins, how many conversions per pin in one cycle will happen, sampling frequency, callback function
    analogContinuous(adc_pins, no_of_instances_, ADC_CONV_PER_PIN, ADC_SAMPLE_RATE, &adcComplete);

    // Start ADC Continuous conversions
    analogContinuousStart();
  }

  /**
   * @brief setup all PWM channels
   */
  static void initPWM(void)
  {
    for (std::uint8_t i = 0; i < no_of_instances_; ++i)
    {
      // Setup timer and attach timer to a led pin
      ledcAttach(instances_[i].pwm_pin, PWM_FREQUENCY, PWM_BIT_WIDTH);
      pinMode(instances_[i].dir_pin, OUTPUT);
    }
  }

  /**
   * @brief set the output voltage for the motor
   */
  void setVoltage(const std::int32_t voltage, const direction_e dir)
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

    ledcWrite(pwm_pins_[instance_index_], duty);
    digitalWrite(dir_pins_[instance_index_], dir);
  }
};
