/*
    (
    )\ ) (
   (()/( )\         (
    /(_)|(_)(    (  )(
   (_))_|_  )\   )\(()\
   | |_ | |((_) ((_)((_)
   | __)| / _ \/ _ \ '_|
   |(|/(|_\___/\___/_)
    )\())  (    ) ( /((        (  (
   ((_)\  ))\( /( )\())\  (    )\))(
    _((_)/((_)(_)|_))((_) )\ )((_))\
   | || (_))((_)_| |_ (_)_(_/( (()(_)
   | __ / -_) _` |  _|| | ' \)) _` |
   |_||_\___\__,_|\__||(|_||_|\__, |
      (      )    (    )\ )   )___/
      )\  ( /((   )\  (()/(( /(
    (((_) )\())( ((_)  /(_))(_))
    )\___(_))(()\ _   (_))((_)
   ((/ __| |_ ((_) |  / __|_  )
    | (__|  _| '_| |  \__ \/ /
     \___|\__|_| |_|  |___/___|

  Floor Heating Controller S2  2024 Alexander <tecnologic86@gmail.com> Evers

  This file is part of Floor Heating Controller S2.

    Floor Heating Controller S2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <cstdint>
#include <algorithm>
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "soc/soc_caps.h"
#include "hardware_layer.h"

namespace hardware
{
  // ESP Logging tag
  const char *TAG = "hardware";

  // TASK notification index
  constexpr std::uint32_t TASK_NOTIFICATION_INDEX = 1;

// Pin for buildin LED
#ifdef S2MINI
  constexpr gpio_num_t LED_PIN = GPIO_NUM_15;
#else
  constexpr gpio_num_t LED_PIN = GPIO_NUM_2;
#endif

  // handle to the control task
  TaskHandle_t task_handle = nullptr;

  /**
   * @brief set the level of the board led
   *
   * @param on true for led on and false for led off
   */
  void SetBoardLed(const bool on)
  {
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(LED_PIN, on));
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
   * @brief set the output voltage for the motor
   *
   * @param channel ledc pwm channel
   * @param u_volt output voltage to set at bridge output
   */
  void updateVoltage(const std::uint8_t channel, const std::int32_t u_volt);

  /**
   * @brief ISR Function that will be triggered when ADC conversion is done
   */
  bool IRAM_ATTR ConvDoneCB(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
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

    ESP_ERROR_CHECK(esp_timer_early_init());

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
          std::uint64_t current_us = esp_timer_get_time();
          static std::uint64_t last_us = 0;
          std::uint64_t diff_us = current_us - last_us;

          for (std::uint8_t i = VALVE_CHAN_1; i < VALVE_CHAN_MAX; ++i)
          {
            valve_controller[i].calculateControls(static_cast<std::uint32_t>(diff_us));
            updateVoltage(i, valve_controller[i].getVoltage());
          }
        }
        /*
        else if (ret == ESP_ERR_TIMEOUT)
        {
          // We try to read  until API returns timeout, which means there's no available data
          ESP_LOGE(TAG, "ADC Read Timeout");
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        */
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
  void ValveController::initADC(void)
  {
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config_, &adc_cali_handle_));
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config_, &adc_handle_));
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle_, &adc_callbacks_, NULL));
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle_, &adc_dig_cfg_));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle_));
  }

  /**
   * @brief setup all PWM channels
   */
  void initPWM(void)
  {
    // Prepare and then apply the LEDC PWM timer configuration
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_config_));

    for (std::uint32_t i = 0; i < VALVE_CHAN_MAX; ++i)
    {

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
      ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    // install fade service for duty and high point update.
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
  }

  /**
   * @brief initialize hardware components
   */
  void Init(void)
  {
    ESP_LOGI(TAG, "Started Hardware Setup");

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask |= (1ULL << LED_PIN);

    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // create the task before to have the task handle ready when adc whats to trigger the task
    xTaskCreate(
        taskControl,   /* Task function. */
        "taskControl", /* String with name of task. */
        10000,         /* Stack size in bytes. */
        NULL,          /* Parameter passed as input of the task */
        1,             /* Priority of the task. */
        &task_handle); /* Task handle. */

    ESP_LOGI(TAG, "Finished Hardware Setup");
  }

  /**
   * @brief set the output voltage for the motor
   *
   * @param channel ledc pwm channel
   * @param u_volt output voltage to set at bridge output
   */
  void updateVoltage(const std::uint8_t channel, const std::int32_t u_volt)
  {
    constexpr std::int32_t PWM_MAX = (1UL << LEDC_TIMER_10_BIT);
    std::int32_t duty = -(PWM_MAX * (u_volt / 1000)) / (SUPPLY_VOLTAGE / 1000);
    std::uint8_t dir = 0;

    duty = std::min(duty, PWM_MAX);

    if (duty < 0)
    {
      duty = PWM_MAX + std::max(duty, -PWM_MAX);
      dir = 1;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel), duty, 0));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(dir_pins[channel], dir));

    // if (channel == 1)
    // {
    //   printf(">duty:%ld\n>hpoint:%ld\n", duty, hpoint);
    // }
  }

  /**
   * @brief standard constructor
   */
  ValveController::ValveController(const gpio_num_t cur_pin,
                                   const gpio_num_t fwd_pin,
                                   const gpio_num_t bwd_pin)
      : // ESP Logging tag
        TAG("valve"),
        // PWM frequency in Hz
        PWM_FREQUENCY(16000),
        // ADC conversions per second
        ADC_CONV_PER_SEC(SOC_ADC_SAMPLE_FREQ_THRES_HIGH),
        // ADC conversions per pin
        ADC_CONV_PER_PIN((ADC_CONV_PER_SEC / PWM_FREQUENCY) * 2),
        // ADC Pin attenuation
        ADC_ATTENUATION(ADC_ATTEN_DB_12),
        // Conversion factor from mV to uA
        MV_TO_UA(70),
        // adc input gpio for current measurement
        ADC_PIN(cur_pin),
        // pwm output gpio for forward rotation.
        FWD_PIN(fwd_pin),
        // pwm output gpio for backwards rotation
        BWD_PIN(bwd_pin),
        // handle to adc instance
        adc_handle_(nullptr),
        // config of the adc unit
        adc_config_{
            .max_store_buf_size = ADC_CONV_PER_PIN * 4 * SOC_ADC_DIGI_RESULT_BYTES,
            .conv_frame_size = ADC_CONV_PER_PIN * SOC_ADC_DIGI_RESULT_BYTES,
        },
        // config of the adcs dig unit
        adc_dig_cfg_{
            .pattern_num = 1,
            .adc_pattern = &adc_pattern_,
            .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_HIGH,
            .conv_mode = ADC_CONV_SINGLE_UNIT_1,
            .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        },
        // adc channel pattern config
        adc_pattern_{
            .atten = ADC_ATTENUATION,
            .channel = ADC_PIN & 0x7,
            .unit = ADC_UNIT_1,
            .bit_width = ADC_BITWIDTH_12,
        },
        // adc calibration config
        cali_config_{
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTENUATION,
            .bitwidth = ADC_BITWIDTH_12,
        },
        // adc calibration handle
        adc_cali_handle_(nullptr),
        // adc callbacks configuration
        adc_callbacks_{
            .on_conv_done = ConvDoneCB,
            .on_pool_ovf = nullptr,
        },
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_{
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_10_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = PWM_FREQUENCY,
            .clk_cfg = LEDC_AUTO_CLK,
        },
        // pwm channel for forward rotation configuration
        ledc_channel_fwd_{
            .gpio_num = FWD_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = (1U << ledc_timer_config_.duty_resolution) / 2,
            .hpoint = (1U << ledc_timer_config_.duty_resolution) / 2,
            .flags = {0},
        },
        // pwm channel for backward rotation configuration
        ledc_channel_bwd_{
            .gpio_num = BWD_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_1,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = (1U << ledc_timer_config_.duty_resolution) / 2,
            .hpoint = (1U << ledc_timer_config_.duty_resolution) / 2,
            .flags = {0},
        },
        // current measurement in uA
        actual_current_(0),
        // set current in uA
        set_current_(0),
        // current offset in uA
        offset_current_(0),
        // output voltage in uV
        output_voltage_(0),
        // offset samples needed just after boot
        offset_needed_(true),
        // actual position
        actual_position_(0),
        // set position
        set_position_(0),
        // set speed
        set_speed_(0),
        // reference speed
        ref_speed_(0),
        // actual speed
        actual_speed_(0),
        // acceleration
        acceleration_(10000),
        // homing current
        homing_current_(25000),
        // homing speed
        homing_speed_(1000000),
        // homing direction
        homing_direction_(false),
        // homing timeout
        homing_timeout_(10000),
        // current readings sum
        current_reading_sum_(0),
        // current reading count
        current_reading_count_(0),
        // Motor series resistance mOhm
        series_resistance_(42000),
        // Motor series inductance uH
        series_inductance_(16300),
        // Pi Controller for motor current
        ctrl_current_(0, 1000, SUPPLY_VOLTAGE, -SUPPLY_VOLTAGE),
        // Pi Controller for motor speed
        ctrl_speed_(0, 100000, 100000, -100000),
        // Pi Controller for motor position
        ctrl_position_(0, 0, 100000, 100000){};

  /**
   * @brief Try to home the valve
   *
   * @retval true for successful competition of the homeing run. false for timeout
   */
  bool ValveController::home(void)
  {
    return false;
  }

  /**
   * @brief calculate the control loops
   *
   * @param Tms time since last call in micro seconds
   */
  void ValveController::calculateControls(std::uint32_t Tus)
  {
    if (offset_needed_)
    {
      if (current_reading_count_ > OFFSET_SAMPLES)
      {
        offset_current_ = getReading();
        offset_needed_ = false;
      }
    }
    else
    {
      std::int32_t last_current = actual_current_;
      actual_current_ = getReading() - offset_current_;

      std::int32_t ff = 0; //(set_current_ * series_resistance_) / 1000;

      output_voltage_ = pi_current_.Calculate(set_current_, actual_current_, ff, Tus);

      if (std::abs(actual_current_) > 1000)
      {
        // voltage across the series resistance in uV
        std::int32_t r_volt = (actual_current_ * series_resistance_) / 1000;
        // u = L * di /dt
        std::int32_t i_volt = (series_inductance_ * (actual_current_ - last_current)) / Tus;

        // Bemf voltage as measurement for speed
        actual_speed_ = output_voltage_ - (r_volt + i_volt);

        // integrate to position
        actual_position_ += (actual_speed_ * 1000) / static_cast<std::int32_t>(Tus);
      }
      else
      {
        actual_speed_ = 0;
      }
    }
  }

  // valve controller instances
  std::array<ValveController, VALVE_CHAN_MAX> valve_controller;

} /* namespace hardware */
