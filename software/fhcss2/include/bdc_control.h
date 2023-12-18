#ifndef BDC_SENSORLESS_POSITION_CONTROL_
#define BDC_SENSORLESS_POSITION_CONTROL_

#include <stdio.h>
#include <cstdint>
#include <algorithm>
#include <array>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"



/**
 * @brief class controles brushed dc motor with sensorless position control
 */
template <std::uint32_t N_MAX_ = 8>
class BdcSensorlessPositionControl
{
public:
  // PWM supply voltage in mV
  static constexpr std::int32_t PWM_VOLTAGE = 5000; // supply voltage in mv
  // ADC sample rate
  static constexpr std::uint32_t ADC_SAMPLE_RATE = 80000;
  // ADC conversions per pin
  static constexpr std::uint32_t ADC_CONV_PER_PIN = 30;
  // Conversion factor from mV to uA
  static constexpr std::uint32_t MV_TO_UA = 70;
  // Number of offset samples
  static constexpr std::uint32_t OFFSET_SAMPLES = 1000;
  // TASK notification index
  static constexpr std::uint32_t TASK_NOTIFICATION_INDEX = 1;
  // Pin for buildin LED
  #ifdef S2MINI
  static constexpr gpio_num_t LED_PIN = GPIO_NUM_15;
  #else
  static constexpr gpio_num_t LED_PIN = GPIO_NUM_2;
  #endif

protected:
  // ESP Logging tag
  static const char* TAG;

  // array with all the class instances
  static std::array<BdcSensorlessPositionControl *, N_MAX_> instances_;
  // number of elements in instances_ array
  static std::uint32_t no_of_instances_;

  // handle to the control task
  static TaskHandle_t taskHandle_;

  // // ISR Function that will be triggered when ADC conversion is done
  // static inline void ARDUINO_ISR_ATTR adcComplete(void)
  // {
  //   BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  //   /* Notify the task that the transmission is complete. */
  //   vTaskNotifyGiveFromISR(taskHandle_,
  //                          &xHigherPriorityTaskWoken);
  //   /* If xHigherPriorityTaskWoken is now set to pdTRUE then a
  //   context switch should be performed to ensure the interrupt
  //   returns directly to the highest priority task.  The macro used
  //   for this purpose is dependent on the port in use and may be
  //   called portEND_SWITCHING_ISR(). */
  //   portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  // }

  static void taskControl(void *parameter)
  {
    while (1)
    {
      uint32_t notification_value = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(5000));

      if (notification_value)
      {
        // // ADC Conversion was done
        // // Read data from ADC
        // if (analogContinuousRead(&adc_result_, 0))
        // {
        //   for (int i = 0; i < no_of_instances_; i++)
        //   {
        //     if (instances_[i]->offset_samples_ < OFFSET_SAMPLES)
        //     {
        //       instances_[i]->offset_current_ += adc_result_[i].avg_read_mvolts * MV_TO_UA;
        //       instances_[i]->offset_samples_++;
        //     }
        //     else if (instances_[i]->offset_samples_ == OFFSET_SAMPLES)
        //     {
        //       instances_[i]->offset_current_ /= instances_[i]->offset_samples_;
        //       instances_[i]->offset_samples_++;
        //       Serial.printf("Motor %d current offset %d uA\n", i, instances_[i]->offset_current_);
        //     }
        //     else
        //     {
        //       instances_[i]->actual_current_ = adc_result_[i].avg_read_mvolts * MV_TO_UA - instances_[i]->offset_current_;
        //     }
        //   }
        // }
        // else
        // {
        //   ESP_LOGE(TAG, "Error occured during reading adc data.");
        // }
      }
      else
      {
        // Timeout
        ESP_LOGW(TAG, "ADC Timeout");
      }
    }
  }

  // index of the instance
  std::uint32_t index_of_instance_;
  // current measurement adc input pin
  gpio_num_t adc_pin_;
  // pwm output for the motor
  gpio_num_t pwm_pin_;
  // 2nd output to change direction of rotation of the motor
  gpio_num_t dir_pin_;
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
      const gpio_num_t pwm_pin = 9,
      const gpio_num_t dir_pin = 10,
      const gpio_num_t adc_pin = 1) : index_of_instance_(no_of_instances_),
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
    
  }

  /**
   * @brief setup all PWM channels
   */
  static void initPWM(void)
  {
     //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE,
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask |= (1ULL << LED_PIN);
        

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_12_BIT ,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 1000,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    for (std::uint32_t i = 0; i < no_of_instances_; ++i)
    {
        //bit mask of the pins that you want to set
        io_conf.pin_bit_mask |= (1ULL << instances_[i]->dir_pin_);
        // io_conf.pin_bit_mask |= (1ULL << instances_[i]->pwm_pin_);

        ESP_LOGI(TAG, "Init of PWM Channel %lu", i);
        // Prepare and then apply the LEDC PWM channel configuration
        ledc_channel_config_t ledc_channel = {
            .gpio_num       = instances_[i]->pwm_pin_,
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = (ledc_channel_t)i,
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 250, // Set duty to 0%
            .hpoint         = 0,
            .flags          = {0}
        };
        ledc_channel_config(&ledc_channel);
    }

    //configure GPIO with the given settings
    gpio_config(&io_conf);
  }

  /**
   * @brief set the output voltage for the motor
   */
  void setVoltage(const std::int32_t voltage)
  {
    constexpr std::int32_t PWM_MAX = (1UL << LEDC_TIMER_12_BIT);
    std::int32_t duty = (PWM_MAX  * voltage) / PWM_VOLTAGE;
    std::uint8_t dir = 0;
  
    duty = std::min(duty, PWM_MAX);

    if (duty < 0)
    {
      duty = PWM_MAX + std::max(duty, -PWM_MAX);
      dir = 1;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)index_of_instance_, duty);
    // Update duty to apply the new value
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)index_of_instance_);
    gpio_set_level(dir_pin_, dir);

    ESP_LOGD(TAG, "Motor %lu Current %ld uA, Set Voltage %ld mV, Duty %ld, Dir %u", index_of_instance_, getCurrent(), voltage, duty, dir);
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

// handle to the control task
template <std::uint32_t N_MAX_>
TaskHandle_t BdcSensorlessPositionControl<N_MAX_>::taskHandle_ = nullptr;

// ESP Logging tag
template <std::uint32_t N_MAX_>
const char*  BdcSensorlessPositionControl<N_MAX_>::TAG = "bdc";

#endif /* BDC_SENSORLESS_POSITION_CONTROL_ */
