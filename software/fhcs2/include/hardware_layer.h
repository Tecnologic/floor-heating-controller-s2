#ifndef _HARDWARE_LAYER_H_
#define _HARDWARE_LAYER_H_

#include "controller.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_filter.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <array>
#include <cstdint>

namespace hardware
{
  /**
   * @brief set the level of the board led
   *
   * @param on true for led on and false for led off
   */
  extern void SetBoardLed(const bool on);

  /**
   * @brief get the level of the board led
   *
   * @retval true for led on and false for led off
   */
  extern bool GetBoardLed(void);

  /**
   * @brief ISR Function that will be triggered when ADC conversion is done
   */
  extern bool ConvDoneCB(adc_continuous_handle_t handle,
                         const adc_continuous_evt_data_t *edata, void *user_data);

  // Supply Voltage of the power bridge in uV
  constexpr std::int32_t SUPPLY_VOLTAGE = 5000;

  /**
   * @brief Valve drive position controller
   *
   * The controller will apply a trapezodial speed profile
   * in order to do the path planning for a smooth motion.
   *
   * The position controller will command a new set speed
   * for the drive the rate of change of that set speed
   * is limited by the ramp rate.
   */
  class ValveController
  {
  public:
    // Channel definitions
    enum channels_e
    {
      CHANNEL_1,
      CHANNEL_2,
      CHANNEL_3,
      CHANNEL_4,
      CHANNEL_5,
      CHANNEL_6,
      CHANNEL_7,
      CHANNEL_8,
      CHANNEL_MAX,
    };
    /**
     * @brief get the actual voltage supplied to the motor
     *
     * @retval actual voltage output of the bridge to the motor in micro Volts
     */
    inline std::uint32_t getVoltage(void) { return (output_voltage_); };

    inline void setVoltage(const std::uint32_t voltage)
    {
      output_voltage_ = voltage;
    };

    /**
     * @brief get the actual current of the motor
     *
     * @retval actual current of the motor
     */
    inline std::int32_t getCurrent(void) { return (actual_current_); };

    /**
     * @brief get the set current of the motor.
     *
     * @retval current setpoint
     */
    inline std::int32_t getSetCurrent(void) { return (set_current_); };

    /**
     * @brief set the set current of the motor.
     *
     * @param current setpoint for the current controller
     */
    inline void setSetCurrent(const std::int32_t current)
    {
      set_current_ = current;
    };

    /**
     * @brief set the current position setpoint of the valve
     *
     * @param position new position setpoint
     */
    inline void setPosition(const std::int32_t position)
    {
      set_position_ = position;
    };

    /**
     * @brief get the current position setpoint
     * @retval current position setpoint
     */
    inline std::int32_t getSetPosition(void) { return (set_position_); };

    /**
     * @brief get the actual position of the valve
     *
     * @note during motion this value will be different from the
     *       position setpoint, because the drive will not jump to
     *       the next position.
     *
     * @retval actual position of the valve
     */
    inline std::int32_t getPosition(void) { return (actual_position_); };

    /**
     * @brief get the actual speed setpoint
     *
     * @retval actual set speed, output of position control loop
     */
    inline std::int32_t getSetSpeed(void) { return (set_speed_); };

    /**
     * @brief set the actual speed setpoint
     *
     * @param speed new set speed
     */
    inline void setSetSpeed(const std::int32_t speed) { set_speed_ = speed; };

    /**
     * @brief get the actual speed reference value
     *
     * @retval actual reference speed
     */
    inline std::int32_t getRefSpeed(void) { return (ref_speed_); };

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
    inline std::int32_t getSpeed(void) { return (actual_speed_); };

    /**
     * @brief get the speed ramp
     *
     * @retval actual speed ramp
     */
    inline std::int32_t getAcceleration(void) { return (acceleration_); };

    /**
     * @brief set the speed ramp
     *
     * @param ramp speed ramp
     */
    inline void setAcceleration(const std::int32_t acc) { acceleration_ = acc; };

    /**
     * @brief set the current limit for homing
     *
     * @param current new homing current limit
     */
    inline void setHomingCurrent(const std::int32_t current)
    {
      homing_current_ = current;
    };

    /**
     * @brief set the direction for homing
     *
     * @param forward true for forward rotaton and false for backwards rotation
     */
    inline void setHomingDirection(const bool forward)
    {
      homing_direction_ = forward;
    };

    /**
     * @brief set the timeout for a homing try
     *
     * @param timeout homing timeout in ms
     */
    inline void setHomingTimeout(const std::uint32_t timeout)
    {
      homing_timeout_ = timeout;
    };

    /**
     * @brief set homing speed
     *
     * @param speed new homing speed
     */
    inline void setHomingSpeed(const std::int32_t speed)
    {
      homing_speed_ = speed;
    };

    /**
     * @brief get the current limit for homing
     *
     * @retval actual homing current limit
     */
    inline std::int32_t getHomingCurrent(void) { return (homing_current_); };

    /**
     * @brief get the direction for homing
     *
     * @retval true for forward rotaton and false for backwards rotation
     */
    inline bool getHomingDirection(void) { return (homing_direction_); };

    /**
     * @brief get homing speed
     *
     * @retval actual homing speed
     */
    inline std::int32_t getHomingSpeed(void) { return (homing_speed_); };

    /**
     * @brief get the timeout for a homing try
     *
     * @retval homing timeout in ms
     */
    inline std::uint32_t getHomingTimeout(void) { return (homing_timeout_); };

    /**
     * @brief set the series resistance of the motor in mOhms
     *
     * @param mOhms series resistance of the motor in milli ohms
     */
    inline void setMotorSeriesResitance(const std::int32_t mOhms)
    {
      series_resistance_ = mOhms;
    };

    /**
     * @brief get the series resistance of the motor in mOhms
     *
     * @retval series resistance of the motor in milli ohms
     */
    inline std::int32_t getMotorSeriesResitance(void)
    {
      return (series_resistance_);
    };

    /**
     * @brief set the series inductance of the motor in micro henry
     *
     * @param uH series inductance of the motor in micro henry
     */
    inline void setMotorSeriesInductance(const std::int32_t uH)
    {
      series_inductance_ = uH;
    };

    /**
     * @brief get the series inductance of the motor in micro henry
     *
     * @retval series inductance of the motor in micro henry
     */
    inline std::int32_t getMotorSeriesInductance(void)
    {
      return (series_inductance_);
    };

    /**
     * @brief Try to home the valve
     *
     * @retval true for successful competition of the homeing run. false for
     * timeout
     */
    bool home(void);

    /**
     * @brief command a new position to the drive and move it there
     *
     * @param position new position setpoint for the drive
     * @return true move is completed / nothing to do
     * @return false currently busy, doing the requested move
     */
    bool move(const std::int32_t position);

    /**
     * @brief auire control of the drives hardware
     *
     * @param forward true on forward rotation and false for back wards rotation
     */
    void acquire(bool forward);

    /**
     * @brief release the control of the drives hardware
     *
     */
    void release(void);

    /**
     * @brief standard constructor
     */
    ValveController(const gpio_num_t cur_pin, const gpio_num_t bwd_pin,
                    const gpio_num_t fwd_pin);

  protected:
    // ESP Logging tag
    static const char *TAG;
    // PWM frequency in Hz
    static constexpr std::int32_t PWM_FREQUENCY = 8000;
    // ADC conversions per second
    static constexpr std::uint32_t ADC_CONV_PER_SEC =
        SOC_ADC_SAMPLE_FREQ_THRES_HIGH;
    // ADC conversions per pin
    static constexpr std::uint32_t ADC_CONV_PER_PIN =
        (ADC_CONV_PER_SEC / PWM_FREQUENCY) * 20;
    // ADC Pin attenuation
    static constexpr adc_atten_t ADC_ATTENUATION = ADC_ATTEN_DB_12;
    // Conversion factor from mV to uA
    static constexpr std::int32_t MV_TO_UA = 70;

    // config of the adc unit
    static constexpr adc_continuous_handle_cfg_t ADC_CONFIG = {
        .max_store_buf_size = ADC_CONV_PER_PIN * 4 * SOC_ADC_DIGI_RESULT_BYTES,
        .conv_frame_size = ADC_CONV_PER_PIN * SOC_ADC_DIGI_RESULT_BYTES,
    };
    // adc calibration config
    static constexpr adc_cali_line_fitting_config_t ADC_CALI_CONFIG = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_12,
    };

    // adc callback configuration
    static constexpr adc_continuous_evt_cbs_t ADC_CALLBACKS = {
        .on_conv_done = ConvDoneCB,
        .on_pool_ovf = nullptr,
    };

    // pwm timer configuration
    static constexpr ledc_timer_config_t LEDC_TIMER_CONFIG = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_11_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    // handle to adc instance
    static adc_continuous_handle_t adc_handle_;
    // adc calibration handle
    static adc_cali_handle_t adc_cali_handle_;
    // adc result buffer
    static std::array<uint8_t, ADC_CONFIG.conv_frame_size> adc_result_;
    // mutex to interlock hardware usage
    static SemaphoreHandle_t hardware_mutex_;
    // Reference to the instance of the class which has currently taken that mutex
    static ValveController *active_instance_;
    // Counter of instances
    static std::uint32_t instance_count_;
    // Mask for all Output of the instances
    static uint64_t output_mask_;
    // Mask for all Inputs of the instances
    static uint64_t input_mask_;

    // adc input gpio for current measurement
    const gpio_num_t ADC_PIN;
    // pwm output gpio for forward rotation.
    const gpio_num_t FWD_PIN;
    // pwm output gpio for backwards rotation
    const gpio_num_t BWD_PIN;

    // config of the adcs dig unit
    const adc_continuous_config_t adc_dig_cfg_;
    // adc channel pattern config
    adc_digi_pattern_config_t adc_pattern_;
    // ADC IIR Filter configuration
    adc_continuous_iir_filter_config_t adc_filter_config_;
    // ADC IIR Filter handle
    adc_iir_filter_handle_t adc_filter_;
    // pwm channel for current control
    ledc_channel_config_t ledc_channel_config_;

    // Counter of instances
    std::uint32_t instance_index_;
    // current measurement in uA
    std::int32_t actual_current_;
    // set current in uA
    std::int32_t set_current_;
    // current offset in uA
    std::int32_t offset_current_;
    // output voltage in mV
    std::uint32_t output_voltage_;
    // offset samples needed just after boot
    bool offset_needed_;
    // actual moving direction, true is forward and false is backwards
    bool forward_direction_;
    // actual position
    std::int32_t actual_position_;
    // set position
    std::int32_t set_position_;
    // set speed
    std::int32_t set_speed_;
    // reference speed
    std::int32_t ref_speed_;
    // actual speed
    std::int32_t actual_speed_;
    // acceleration
    std::int32_t acceleration_;
    // homing current
    std::int32_t homing_current_;
    // homing speed
    std::int32_t homing_speed_;
    // homing direction
    bool homing_direction_;
    // homing timeout
    std::uint32_t homing_timeout_;
    // current readings sum
    std::int32_t current_reading_sum_;
    // current reading count
    std::uint32_t current_reading_count_;
    // Motor series resistance 1/10 Ohm
    std::int32_t series_resistance_;
    // Motor series inductance 1/10 mH
    std::int32_t series_inductance_;
    // Pi Controller for motor current
    control::pi ctrl_current_;
    // Pi Controller for motor speed
    control::pi ctrl_speed_;
    // Pi Controller for motor position
    control::pi ctrl_position_;

    /**
     * @brief setup the adc and pwm in general
     *
     * This function does the basic init of the ADC and LEDC pwm unit and needs to
     * be called just once
     */
    static void init(void);

    /**
     * @brief setup the adc for this valve instance and start measuring
     */
    void startADC(void);

    /**
     * @brief stop the adc for this instance
     */
    void stopADC(void);

    /**
     * @brief get a adc reading in micro ampere
     */
    std::int32_t getReading(void);

    /**
     * @brief setup the PWM Channels for this valve instance
     */
    void startPWM(void);

    /**
     * @brief Stop the PWM for this valve instance
     */
    void stopPWM(void);

    /**
     * @brief set the PWM according to the requested output voltage
     */
    void updateVoltage(void);

    /**
     * @brief calculate the control loops
     *
     * @param Tms time since last call in milli seconds
     */
    void calculateControls(const std::uint32_t Tus);

    /**
     * @brief calculate the speed and position by a small emf estimator.
     *
     */
    void calculateMotorModel(const std::uint32_t Tus);

    /* internal friend functions */
    friend void Init(void);
    friend void taskControl(void *parameter);
  };

  /**
   * @brief initialize hardware components
   */
  extern void Init(void);

  extern std::array<ValveController, ValveController::CHANNEL_MAX> valves;
} /* namespace hardware */

#endif /* _HARDWARE_LAYER_H_ */