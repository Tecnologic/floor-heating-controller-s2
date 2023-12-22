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
#include "esp_adc/adc_continuous.h"
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
  static constexpr std::uint32_t ADC_SAMPLE_RATE = 800;
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
  static const char *TAG;

  // array with all the class instances
  static std::array<BdcSensorlessPositionControl *, N_MAX_> instances_;
  // number of elements in instances_ array
  static std::uint32_t no_of_instances_;

  // handle to the control task
  static TaskHandle_t task_handle_;

  // handle to adc instance
  static adc_continuous_handle_t adc_handle_;

  // config of the adc unit
  static adc_continuous_handle_cfg_t adc_config_;

  // config of the adcs dig unit
  static adc_continuous_config_t adc_dig_cfg_;

  // adc channel pattern config
  static adc_digi_pattern_config_t adc_pattern_[SOC_ADC_PATT_LEN_MAX];

  // // ISR Function that will be triggered when ADC conversion is done
  static bool IRAM_ATTR ConvDoneCB(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
  {
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(task_handle_, &mustYield);

    return (mustYield == pdTRUE);
  }

  static void taskControl(void *parameter)
  {
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[ADC_CONV_PER_PIN * N_MAX_] = {0};
    memset(result, 0xcc, ADC_CONV_PER_PIN * N_MAX_);

    while (1)
    {
      /**
       * This is to show you the way to use the ADC continuous mode driver event callback.
       * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
       * However in this example, the data processing (print) is slow, so you barely block here.
       *
       * Without using this event callback (to notify this task), you can still just call
       * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
       */
      uint32_t notification_value = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(5000));

      if (notification_value)
      {
        ret = adc_continuous_read(adc_handle_, result, ADC_CONV_PER_PIN * no_of_instances_, &ret_num, 0);
        if (ret == ESP_OK)
        {
          ESP_LOGI("TASK", "ret is %x, ret_num is %" PRIu32 " bytes", ret, ret_num);
          for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
          {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
            uint32_t chan_num = p->type1.channel;
            uint32_t data = p->type1.data;
            /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
            if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1))
            {
              printf(">CH%lu: %lu\n", chan_num, data);
            }
            else
            {
              ESP_LOGW(TAG, "Invalid data [%s_%" PRIu32 "_%" PRIx32 "]", "1", chan_num, data);
            }
          }
          /**
           * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
           * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
           * usually you don't need this delay (as this task will block for a while).
           */
          vTaskDelay(1);
        }
        else if (ret == ESP_ERR_TIMEOUT)
        {
          // We try to read  until API returns timeout, which means there's no available data
          ESP_LOGE(TAG, "ADC Read Timeout");
          break;
        }
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

    // create the task before to have the task handle ready when adc whats to trigger the task
    xTaskCreate(
        taskControl,    /* Task function. */
        "taskControl",  /* String with name of task. */
        10000,          /* Stack size in bytes. */
        NULL,           /* Parameter passed as input of the task */
        1,              /* Priority of the task. */
        &task_handle_); /* Task handle. */

    initADC();
    initPWM();
  }

  /**
   * @brief setup adc conversion
   */
  static void initADC(void)
  {
    adc_config_ = {
        .max_store_buf_size = 1024,
        .conv_frame_size = ADC_CONV_PER_PIN * no_of_instances_,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config_, &adc_handle_));

    for (int i = 0; i < no_of_instances_; i++)
    {
      adc_pattern_[i].atten = ADC_ATTEN_DB_12;
      adc_pattern_[i].channel = instances_[i]->adc_pin_ & 0x7;
      adc_pattern_[i].unit = ADC_UNIT_1;
      adc_pattern_[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

      ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern_[i].atten);
      ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern_[i].channel);
      ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern_[i].unit);
    }

    adc_dig_cfg_ = {
        .pattern_num = no_of_instances_,
        .adc_pattern = adc_pattern_,
        .sample_freq_hz = ADC_SAMPLE_RATE,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle_, &adc_dig_cfg_));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = ConvDoneCB,
        .on_pool_ovf = nullptr,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle_, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle_));
  }

  /**
   * @brief setup all PWM channels
   */
  static void initPWM(void)
  {
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE,
    // disable pull-up mode
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask |= (1ULL << LED_PIN);

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 4000,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    for (std::uint32_t i = 0; i < no_of_instances_; ++i)
    {
      // bit mask of the pins that you want to set
      io_conf.pin_bit_mask |= (1ULL << instances_[i]->dir_pin_);
      // io_conf.pin_bit_mask |= (1ULL << instances_[i]->pwm_pin_);

      ESP_LOGI(TAG, "Init of PWM Channel %lu", i);
      // Prepare and then apply the LEDC PWM channel configuration
      ledc_channel_config_t ledc_channel = {
          .gpio_num = instances_[i]->pwm_pin_,
          .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = (ledc_channel_t)i,
          .intr_type = LEDC_INTR_DISABLE,
          .timer_sel = LEDC_TIMER_0,
          .duty = 250, // Set duty to 0%
          .hpoint = 0,
          .flags = {0}};
      ledc_channel_config(&ledc_channel);
    }

    // configure GPIO with the given settings
    gpio_config(&io_conf);
  }

  /**
   * @brief set the output voltage for the motor
   */
  void setVoltage(const std::int32_t voltage)
  {
    constexpr std::int32_t PWM_MAX = (1UL << LEDC_TIMER_12_BIT);
    std::int32_t duty = (PWM_MAX * voltage) / PWM_VOLTAGE;
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
TaskHandle_t BdcSensorlessPositionControl<N_MAX_>::task_handle_ = nullptr;

// handle to adc instance
template <std::uint32_t N_MAX_>
adc_continuous_handle_t BdcSensorlessPositionControl<N_MAX_>::adc_handle_;

// config of the adc unit
template <std::uint32_t N_MAX_>
adc_continuous_handle_cfg_t BdcSensorlessPositionControl<N_MAX_>::adc_config_;

// config of the adcs dig unit
template <std::uint32_t N_MAX_>
adc_continuous_config_t BdcSensorlessPositionControl<N_MAX_>::adc_dig_cfg_;

// adc channel pattern config
template <std::uint32_t N_MAX_>
adc_digi_pattern_config_t BdcSensorlessPositionControl<N_MAX_>::adc_pattern_[SOC_ADC_PATT_LEN_MAX];

// ESP Logging tag
template <std::uint32_t N_MAX_>
const char *BdcSensorlessPositionControl<N_MAX_>::TAG = "bdc";

#endif /* BDC_SENSORLESS_POSITION_CONTROL_ */
