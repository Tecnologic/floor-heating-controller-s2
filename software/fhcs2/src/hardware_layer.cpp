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

    Floor Heating Controller S2 is free software: you can redistribute it and/or
  modify it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "hardware_layer.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "soc/soc_caps.h"
#include <algorithm>
#include <cstdint>
#include <stdio.h>
#include <limits>

namespace hardware
{
  // ESP Logging tag
  const char *TAG = "hardware";

  // TASK notification index
  constexpr std::uint32_t TASK_NOTIFICATION_INDEX = 1;

// Pin for builtin LED
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
  bool GetBoardLed(void) { return (gpio_get_level(LED_PIN)); }

  /**
   * @brief setup the adc and pwm in general
   *
   * This function does the basic init of the ADC and LEDC pwm unit and needs to
   * be called just once
   */
  void ValveController::init(void)
  {
    ESP_LOGI(TAG, "Started Init");

    /* Create a mutex type semaphore for shared hardware mutal exclusion */
    hardware_mutex_ = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "Mutex initialized");

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // enable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    // disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = output_mask_;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // set all configured outputs to 0
    for (std::uint8_t i = 0; i < 64; i++)
    {
      if (output_mask_ & (1ULL << i))
      {
        ESP_ERROR_CHECK(gpio_set_level(static_cast<gpio_num_t>(i), 0));
      }
    }

    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = input_mask_;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "GPIOs initialized");

    ESP_ERROR_CHECK(
        adc_cali_create_scheme_line_fitting(&ADC_CALI_CONFIG, &adc_cali_handle_));
    ESP_ERROR_CHECK(adc_continuous_new_handle(&ADC_CONFIG, &adc_handle_));
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(
        adc_handle_, &ADC_CALLBACKS, NULL));
    ESP_LOGI(TAG, "ADC Initialized");

    ESP_ERROR_CHECK(ledc_timer_config(&LEDC_TIMER_CONFIG));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
    ESP_LOGI(TAG, "PWM Initialized");
  }

  /**
   * @brief setup the adc for this valve instance and start measuring
   */
  void ValveController::startADC(void)
  {
    ESP_LOGI(TAG, "Start ADC %lu", instance_index_);
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle_, &adc_dig_cfg_));
    ESP_ERROR_CHECK(adc_new_continuous_iir_filter(adc_handle_, &adc_filter_config_, &adc_filter_));
    ESP_ERROR_CHECK(adc_continuous_iir_filter_enable(adc_filter_));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle_));
  }

  /**
   * @brief stop the adc for this instance
   */
  void ValveController::stopADC(void)
  {
    ESP_LOGI(TAG, "Stop ADC %lu", instance_index_);
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle_));
    ESP_ERROR_CHECK(adc_continuous_iir_filter_disable(adc_filter_));
    ESP_ERROR_CHECK(adc_del_continuous_iir_filter(adc_filter_));
  }

  /**
   * @brief setup the PWM Channels for this valve instance
   */
  void ValveController::startPWM()
  {
    ESP_LOGI(TAG, "Start PWM %lu", instance_index_);
    ledc_channel_config_.flags.output_invert = forward_direction_;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_config_));
    ESP_ERROR_CHECK(gpio_set_level(FWD_PIN, forward_direction_));
  }

  /**
   * @brief Stop the PWM for this valve instance
   */
  void ValveController::stopPWM(void)
  {
    ESP_LOGI(TAG, "Stop ADC %lu", instance_index_);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
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
  bool IRAM_ATTR ConvDoneCB(adc_continuous_handle_t handle,
                            const adc_continuous_evt_data_t *edata,
                            void *user_data)
  {
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(task_handle, &mustYield);

    return (mustYield == pdTRUE);
  }

  /**
   * @brief internal task which takes care of the current, speed and position
   * control loop as well as the adc handling
   */
  void taskControl(void *parameter)
  {
    while (1)
    {
      /**
       * This is to show you the way to use the ADC continuous mode driver event
       * callback. This `ulTaskNotifyTake` will block when the data processing in
       * the task is fast. However in this example, the data processing (print) is
       * slow, so you barely block here.
       *
       * Without using this event callback (to notify this task), you can still
       * just call `adc_continuous_read()` here in a loop, with/without a certain
       * block timeout.
       */
      uint32_t notification_value =
          ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(5000));

      if (notification_value)
      {

        if (ValveController::active_instance_)
        {

          std::uint64_t current_us = esp_timer_get_time();
          static std::uint64_t last_us = 0;
          std::uint64_t diff_us = current_us - last_us;
          last_us = current_us;
          ValveController::active_instance_->calculateControls(diff_us);
        }
      }
      else
      {
        // Timeout
        ESP_LOGW(TAG, "ADC Timeout Instance %lu",
                 ValveController::active_instance_->instance_index_);
      }
    }
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

    // init hr timer for time measurements
    ESP_ERROR_CHECK(esp_timer_early_init());

    // init valvecontroller hardware
    ValveController::init();

    // create the task before to have the task handle ready when adc whats to
    // trigger the task
    xTaskCreate(taskControl,   /* Task function. */
                "taskControl", /* String with name of task. */
                10000,         /* Stack size in bytes. */
                NULL,          /* Parameter passed as input of the task */
                1,             /* Priority of the task. */
                &task_handle); /* Task handle. */

    ESP_LOGI(TAG, "Finished Hardware Setup");
  }

  /**
   * @brief update the output voltage for the motor
   *
   * The PWMs are setup as software wise symetric pwm
   */
  void ValveController::updateVoltage(void)
  {
    const std::uint32_t PWM_MAX = (1UL << (LEDC_TIMER_CONFIG.duty_resolution));
    std::uint32_t duty =
        (PWM_MAX * (output_voltage_)) / (SUPPLY_VOLTAGE);

    if (duty > PWM_MAX)
    {
      duty = PWM_MAX;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty, 0));
  }

  /**
   * @brief get a adc reading in micro ampere
   */
  std::int32_t ValveController::getReading(void)
  {
    esp_err_t ret;
    std::uint32_t ret_num = 0;
    std::int32_t cnt = 0;
    std::int32_t sum = 0;

    ret = adc_continuous_read(adc_handle_, adc_result_._M_elems,
                              adc_result_.size(), &ret_num, 0);
    if (ret == ESP_OK)
    {
      for (std::uint32_t i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
      {
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)&adc_result_[i];
        std::uint32_t chan_num = p->type1.channel;
        std::uint32_t data = p->type1.data;
        /* Check the channel number validation, the data is invalid if the channel
         * num exceed the maximum channel */
        if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1))
        {
          int mv_reading = 0;
          ESP_ERROR_CHECK(
              adc_cali_raw_to_voltage(adc_cali_handle_, data, &mv_reading));
          sum += static_cast<std::int32_t>(mv_reading) * MV_TO_UA;
          cnt++;
        }
        else
        {
          ESP_LOGW(TAG, "Invalid data [%s_%" PRIu32 "_%" PRIx32 "]", "1",
                   chan_num, data);
        }
      }
      return (sum / cnt);
    }
    else
    {
      return (0);
    }
  }
  /**
   * @brief standard constructor
   */
  ValveController::ValveController(const gpio_num_t cur_pin,
                                   const gpio_num_t bwd_pin,
                                   const gpio_num_t fwd_pin)
      : // adc input gpio for current measurement
        ADC_PIN(cur_pin),
        // pwm output gpio for forward rotation.
        FWD_PIN(fwd_pin),
        // pwm output gpio for backwards rotation
        BWD_PIN(bwd_pin),
        // config of the adcs dig unit
        adc_dig_cfg_{
            .pattern_num = 1,
            .adc_pattern = &adc_pattern_,
            .sample_freq_hz = ADC_CONV_PER_SEC,
            .conv_mode = ADC_CONV_SINGLE_UNIT_1,
            .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        },
        // adc channel pattern config
        adc_pattern_{
            .atten = ADC_ATTENUATION,
            .channel = 0,
            .unit = ADC_UNIT_1,
            .bit_width = ADC_BITWIDTH_12,
        },
        // ADC IIR Filter Config
        adc_filter_config_{
            .unit = ADC_UNIT_1,
            .channel = ADC_CHANNEL_0,
            .coeff = ADC_DIGI_IIR_FILTER_COEFF_64,
        },
        // pwm channel for current control configuration
        ledc_channel_config_{
            .gpio_num = BWD_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags = {0},
        },
        // Counter of instances
        instance_index_(instance_count_),
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
        // actual moving direction, true is forward and false is backwards
        forward_direction_(true),
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
        // Motor series resistance 1/10 Ohm
        series_resistance_(420),
        // Motor series inductance 1/10 mH
        series_inductance_(163),
        // Pi Controller for motor current
        ctrl_current_(5, 1000, SUPPLY_VOLTAGE - 1000, 0),
        // Pi Controller for motor speed
        ctrl_speed_(0, 100000, 100000, -100000),
        // Pi Controller for motor position
        ctrl_position_(0, 0, 100000, 100000)
  {
    adc_unit_t unit_id;
    adc_channel_t channel;

    instance_count_++;
    output_mask_ |= (1ULL << BWD_PIN) | (1ULL << FWD_PIN);
    input_mask_ |= (1ULL << ADC_PIN);

    adc_continuous_io_to_channel(ADC_PIN, &unit_id, &channel);
    adc_pattern_.channel = static_cast<std::uint8_t>(channel);
    adc_filter_config_.channel = channel;
  };

  /**
   * @brief Try to home the valve
   *
   * @retval true for successful competition of the homeing run. false for timeout
   */
  bool ValveController::home(void) { return false; }

  /**
   * @brief command a new position to the drive and move it there
   *
   * @param position new position setpoint for the drive
   * @return true move is completed / nothing to do
   * @return false currently busy, doing the requested move
   */
  bool ValveController::move(const std::int32_t position)
  {
    bool busy = false;

    if (active_instance_ == nullptr)
    {
      // equal is not handled be cause then no move is nessesary
      if (position > set_position_)
      {
        acquire(true);
      }
      else if (position < set_position_)
      {
        acquire(false);
      }
    }

    if (active_instance_ == this)
    {
      busy = true;
      set_position_ = position;
      if (actual_position_ == set_position_)
      {
        release();
      }
    }
    return busy;
  }

  /**
   * @brief auire control of the drives hardware
   *
   * @param forward true on forward rotation and false for back wards rotation
   */
  void ValveController::acquire(bool forward)
  {
    if (xSemaphoreTake(hardware_mutex_, portMAX_DELAY) == pdTRUE)
    {
      ESP_LOGI(TAG, "Acquired Instance %lu", instance_index_);
      forward_direction_ = forward;
      active_instance_ = this;
      active_instance_->startADC();
      active_instance_->startPWM();
    }
  }

  /**
   * @brief release the control of the drives hardware
   *
   */
  void ValveController::release(void)
  {
    if (active_instance_ == this)
    {
      active_instance_->stopADC();
      active_instance_->stopPWM();
      active_instance_ = nullptr;
      xSemaphoreGive(hardware_mutex_);
      ESP_LOGI(TAG, "Released Instance %lu", instance_index_);
    }
  }

  /**
   * @brief calculate the speed and position by a small emf estimator.
   *
   */
  void ValveController::calculateMotorModel(const std::uint32_t Tus)
  {
    static std::int32_t last_current = 0;

    if (std::abs(actual_current_) > 100)
    {
      // voltage across the series resistance in uV
      std::uint32_t r_volt = (actual_current_ * series_resistance_) / 10000;
      // u = L * di /dt
      std::int32_t i_volt =
          (series_inductance_ * (actual_current_ - last_current)) / (Tus * 10);

      // Bemf voltage as measurement for speed
      actual_speed_ = output_voltage_ - (r_volt + i_volt);

      // integrate to position
      actual_position_ += (actual_speed_ * 10) / static_cast<std::int32_t>(Tus);
    }
    else
    {
      actual_speed_ = 0;
    }

    last_current = actual_current_;
  }

  /**
   * @brief calculate the control loops
   *
   * @param Tms time since last call in micro seconds
   */
  void ValveController::calculateControls(const std::uint32_t Tus)
  {
    if (offset_needed_)
    {
      offset_current_ = getReading();
      offset_needed_ = false;
    }
    else
    {
      actual_current_ = getReading() - offset_current_;
      std::int32_t ff = 0; //(set_current_ * series_resistance_) / 10000;

      calculateMotorModel(Tus);

      output_voltage_ =
          ctrl_current_.Calculate(set_current_, actual_current_, ff, Tus);

      // set_current_ = ctrl_speed_.Calculate(set_speed_, actual_speed_, 0, Tus);
      // set_speed_ =
      //     ctrl_position_.Calculate(set_position_, actual_position_, 0, Tus);
      updateVoltage();
    }
  }

  std::array<ValveController, ValveController::CHANNEL_MAX> valves({
      // current      forward     backward
      {GPIO_NUM_1, GPIO_NUM_9, GPIO_NUM_10},
      {GPIO_NUM_2, GPIO_NUM_11, GPIO_NUM_12},
      {GPIO_NUM_3, GPIO_NUM_13, GPIO_NUM_14},
      {GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17},
      {GPIO_NUM_5, GPIO_NUM_18, GPIO_NUM_19},
      {GPIO_NUM_6, GPIO_NUM_33, GPIO_NUM_34},
      {GPIO_NUM_7, GPIO_NUM_35, GPIO_NUM_36},
      {GPIO_NUM_8, GPIO_NUM_37, GPIO_NUM_38},
  });

  // ESP Logging tag
  const char *ValveController::TAG = "valve";
  // Counter of instances
  std::uint32_t ValveController::instance_count_ = 1;
  // handle to adc instance
  adc_continuous_handle_t ValveController::adc_handle_ = nullptr;
  // adc calibration handle
  adc_cali_handle_t ValveController::adc_cali_handle_ = nullptr;
  // adc result buffer
  std::array<uint8_t, ValveController::ADC_CONFIG.conv_frame_size>
      ValveController::adc_result_;
  // mutex to interlock hardware usage
  SemaphoreHandle_t ValveController::hardware_mutex_;
  // Reference to the instance of the class which has currently taken that mutex
  ValveController *ValveController::active_instance_ = nullptr;
  // Mask for all Output of the instances
  uint64_t ValveController::output_mask_ = 0ULL;
  // Mask for all Inputs of the instances
  uint64_t ValveController::input_mask_ = 0ULL;

} /* namespace hardware */
