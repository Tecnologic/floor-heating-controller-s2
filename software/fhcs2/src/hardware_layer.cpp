#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <cstdint>
#include <algorithm>
#include "esp_attr.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hardware_layer.h"

namespace hardware
{
  // ESP Logging tag
  const char *TAG = "hardware";
  // PWM frequency in Hz
  constexpr std::int32_t PWM_FREQUENCY = 20000;
  // ADC sample rate in Hz
  constexpr std::uint32_t ADC_SAMPLE_RATE = 80000;
  // ADC conversions per pin
  constexpr std::uint32_t ADC_CONV_PER_PIN = 30;
  // ADC Pin attenuation
  constexpr adc_atten_t ADC_ATTENUATION = ADC_ATTEN_DB_6;
  // Conversion factor from mV to uA
  constexpr std::uint32_t MV_TO_UA = 139;
  // Number of offset samples
  constexpr std::uint32_t OFFSET_SAMPLES = 1000;
  // TASK notification index
  constexpr std::uint32_t TASK_NOTIFICATION_INDEX = 1;

  // List of adc input gpios
  constexpr std::array adc_pins{GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_8};

  // List of pwm output gpios
  constexpr std::array pwm_pins{GPIO_NUM_9, GPIO_NUM_11, GPIO_NUM_13, GPIO_NUM_16, GPIO_NUM_18, GPIO_NUM_33, GPIO_NUM_35, GPIO_NUM_37};

  // List of dir output gpios
  constexpr std::array dir_pins{GPIO_NUM_10, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_17, GPIO_NUM_19, GPIO_NUM_34, GPIO_NUM_36, GPIO_NUM_38};

// Pin for buildin LED
#ifdef S2MINI
  constexpr gpio_num_t LED_PIN = GPIO_NUM_15;
#else
  constexpr gpio_num_t LED_PIN = GPIO_NUM_2;
#endif

  // handle to the control task
  TaskHandle_t task_handle = nullptr;

  // handle to adc instance
  adc_continuous_handle_t adc_handle = nullptr;

  // config of the adc unit
  adc_continuous_handle_cfg_t adc_config;

  // config of the adcs dig unit
  adc_continuous_config_t adc_dig_cfg;

  // adc channel pattern config
  std::array<adc_digi_pattern_config_t, VALVE_CHAN_MAX> adc_pattern;

  // adc calibration handle
  adc_cali_handle_t adc_cali_handle = nullptr;

  // offset samples for current offsets
  std::uint32_t adc_offset_samples = 0;

  /**
   * @brief set the level of the board led
   *
   * @param on true for led on and false for led off
   */
  void SetBoardLed(const bool on)
  {
    ESP_ERROR_CHECK(gpio_set_level(LED_PIN, on));
  }

  /**
   * @brief get the level of the board led
   *
   * @retval true for led on and false for led off
   */
  bool GetBoardLed(void)
  {
    return (gpio_get_level(LED_PIN));
  }

  /**
   * @brief ISR Function that will be triggered when ADC conversion is done
   */
  bool IRAM_ATTR
  ConvDoneCB(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
  {
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(task_handle, &mustYield);

    return (mustYield == pdTRUE);
  }

  /**
   * @brief internal task which takes care of the current, speed and position control loop as well as the adc handling
   */
  void taskControl(void *parameter)
  {
    esp_err_t ret;
    std::uint32_t ret_num = 0;
    std::array<uint8_t, ADC_CONV_PER_PIN * VALVE_CHAN_MAX> result = {0};
    result.fill(0xcc);

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
        ret = adc_continuous_read(adc_handle, result._M_elems, ADC_CONV_PER_PIN * VALVE_CHAN_MAX, &ret_num, 0);
        if (ret == ESP_OK)
        {
          // ESP_LOGI("TASK", "ret is %x, ret_num is %" PRIu32 " bytes", ret, ret_num);
          for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
          {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
            uint32_t chan_num = p->type1.channel;
            uint32_t data = p->type1.data;
            /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
            if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1))
            {
              int mv_reading = 0;
              ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, data, &mv_reading));
              valve_controller[chan_num].addReading(mv_reading * MV_TO_UA);
            }
            else
            {
              ESP_LOGW(TAG, "Invalid data [%s_%" PRIu32 "_%" PRIx32 "]", "1", chan_num, data);
            }
          }
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

  /**
   * @brief setup adc conversion
   */
  void initADC(void)
  {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_13,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle));

    adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = ADC_CONV_PER_PIN * VALVE_CHAN_MAX,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    for (int i = 0; i < VALVE_CHAN_MAX; i++)
    {
      adc_pattern[i].atten = ADC_ATTENUATION;
      adc_pattern[i].channel = adc_pins[i] & 0x7;
      adc_pattern[i].unit = ADC_UNIT_1;
      adc_pattern[i].bit_width = ADC_BITWIDTH_13;

      ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
      ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
      ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
    }

    adc_dig_cfg = {
        .pattern_num = VALVE_CHAN_MAX,
        .adc_pattern = adc_pattern._M_elems,
        .sample_freq_hz = ADC_SAMPLE_RATE,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_dig_cfg));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = ConvDoneCB,
        .on_pool_ovf = nullptr,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
  }

  /**
   * @brief setup all PWM channels
   */
  void initPWM(void)
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
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    for (std::uint32_t i = 0; i < VALVE_CHAN_MAX; ++i)
    {
      // bit mask of the pins that you want to set
      io_conf.pin_bit_mask |= (1ULL << dir_pins[i]);

      ESP_LOGI(TAG, "Init of PWM Channel %lu", i);
      // Prepare and then apply the LEDC PWM channel configuration
      ledc_channel_config_t ledc_channel = {
          .gpio_num = pwm_pins[i],
          .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = (ledc_channel_t)i,
          .intr_type = LEDC_INTR_DISABLE,
          .timer_sel = LEDC_TIMER_0,
          .duty = 0,
          .hpoint = 0,
          .flags = {0}};
      ledc_channel_config(&ledc_channel);
    }

    // configure GPIO with the given settings
    gpio_config(&io_conf);
  }

  /**
   * @brief initialize hardware components
   */
  void Init(void)
  {
    ESP_LOGI(TAG, "Started Hardware Setup");
    // create the task before to have the task handle ready when adc whats to trigger the task
    xTaskCreate(
        taskControl,   /* Task function. */
        "taskControl", /* String with name of task. */
        10000,         /* Stack size in bytes. */
        NULL,          /* Parameter passed as input of the task */
        1,             /* Priority of the task. */
        &task_handle); /* Task handle. */

    initADC();
    initPWM();
    ESP_LOGI(TAG, "Finished Hardware Setup");
  }

  /**
   * @brief set the current position setpoint of the valve
   *
   * @param position new position setpoint
   */
  void ValveController::setPosition(const std::int32_t position)
  {
    set_position_ = position;
  }

  /**
   * @brief get the current position setpoint
   * @retval current position setpoint
   */
  std::int32_t ValveController::getSetPosition(void)
  {
    return (set_position_);
  }

  /**
   * @brief get the actual position of the valve
   *
   * @note during motion this value will be different from the
   *       position setpoint, because the drive will not jump to
   *       the next position.
   *
   * @retval actual position of the valve
   */
  std::int32_t ValveController::getPosition(void)
  {
    return (actual_position_);
  }

  /**
   * @brief get the actual speed setpoint
   *
   * @retval actual set speed, output of position control loop
   */
  std::int32_t ValveController::getSetSpeed(void)
  {
    return (set_speed_);
  }

  /**
   * @brief get the actual speed reference value
   *
   * @retval actual reference speed
   */
  std::int32_t ValveController::getRefSpeed(void)
  {
    return (ref_speed_);
  }

  /**
   * @brief get the actual speed of the motor
   *
   * The controller will apply a trapezodial speed profile
   * in order to do the path planning for a smooth motion.
   *
   * @note the position controller will command a new set speed
   *       for the drive the rate of change of that set speed
   *       is limited by the ramp rate.
   *
   * @retval actual speed of the motor
   */
  std::int32_t ValveController::getSpeed(void)
  {
    return (actual_speed_);
  }

  /**
   * @brief get the speed ramp
   *
   * @retval actual speed ramp
   */
  std::int32_t ValveController::getAcceleration(void)
  {
    return (acceleration_);
  }

  /**
   * @brief set the speed ramp
   *
   * @param ramp speed ramp
   */
  void ValveController::setAcceleration(const std::int32_t acceleration)
  {
    acceleration_ = acceleration;
  }

  /**
   * @brief set the current limit for homing
   *
   * @param current new homing current limit
   */
  void ValveController::setHomingCurrent(const std::int32_t current)
  {
    homing_current_ = current;
  }

  /**
   * @brief set the direction for homing
   *
   * @param forward true for forward rotaton and false for backwards rotation
   */
  void ValveController::setHomingDirection(const bool forward)
  {
    homing_direction_ = forward;
  }

  /**
   * @brief set the timeout for a homing try
   *
   * @param timeout homing timeout in ms
   */
  void ValveController::setHomingTimeout(const std::uint32_t timeout)
  {
    homing_timeout_ = timeout;
  }

  /**
   * @brief set homing speed
   *
   * @param speed new homing speed
   */
  void ValveController::setHomingSpeed(const std::int32_t speed)
  {
    homing_speed_ = speed;
  }

  /**
   * @brief get the current limit for homing
   *
   * @retval actual homing current limit
   */
  std::int32_t ValveController::getHomingCurrent(void)
  {
    return (homing_current_);
  }

  /**
   * @brief get the direction for homing
   *
   * @retval true for forward rotaton and false for backwards rotation
   */
  bool ValveController::getHomingDirection(void)
  {
    return (homing_direction_);
  }

  /**
   * @brief get homing speed
   *
   * @retval actual homing speed
   */
  std::int32_t ValveController::getHomingSpeed(void)
  {
    return (homing_speed_);
  }

  /**
   * @brief get the timeout for a homing try
   *
   * @retval homing timeout in ms
   */
  std::uint32_t ValveController::getHomingTimeout(void)
  {
    return (homing_timeout_);
  }

  /**
   * @brief Try to home the valve
   *
   * @retval true for successful competition of the homeing run. false for timeout
   */
  bool ValveController::home(void)
  {
    return false;
  }

  // valve controller instances
  std::array<ValveController, VALVE_CHAN_MAX> valve_controller;

} /* namespace hardware */
