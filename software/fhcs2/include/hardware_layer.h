#ifndef _HARDWARE_LAYER_H_
#define _HARDWARE_LAYER_H_

#include <cstdint>
#include <array>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "controller.h"

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

  // Supply Voltage of the power bridge in uV
  constexpr std::int32_t SUPPLY_VOLTAGE = 5000000;

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
    /**
     * @brief get the actual voltage supplied to the motor
     *
     * @retval actual voltage output of the bridge to the motor in micro Volts
     */
    inline std::int32_t getVoltage(void) { return (output_voltage_); };

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
    inline void setSetCurrent(const std::int32_t current) { set_current_ = current; };

    /**
     * @brief set the current position setpoint of the valve
     *
     * @param position new position setpoint
     */
    inline void setPosition(const std::int32_t position) { set_position_ = position; };

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
    inline std::int32_t setSetSpeed(const std::int32_t speed) { set_speed_ = speed; };

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
    inline void setHomingCurrent(const std::int32_t current) { homing_current_ = current; };

    /**
     * @brief set the direction for homing
     *
     * @param forward true for forward rotaton and false for backwards rotation
     */
    inline void setHomingDirection(const bool forward) { homing_direction_ = forward; };

    /**
     * @brief set the timeout for a homing try
     *
     * @param timeout homing timeout in ms
     */
    inline void setHomingTimeout(const std::uint32_t timeout) { homing_timeout_ = timeout; };

    /**
     * @brief set homing speed
     *
     * @param speed new homing speed
     */
    inline void setHomingSpeed(const std::int32_t speed) { homing_speed_ = speed; };

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
    inline void setMotorSeriesResitance(const std::int32_t mOhms) { series_resistance_ = mOhms; };

    /**
     * @brief get the series resistance of the motor in mOhms
     *
     * @retval series resistance of the motor in milli ohms
     */
    inline std::int32_t getMotorSeriesResitance(void) { return (series_resistance_); };

    /**
     * @brief set the series inductance of the motor in micro henry
     *
     * @param uH series inductance of the motor in micro henry
     */
    inline void setMotorSeriesInductance(const std::int32_t uH) { series_inductance_ = uH; };

    /**
     * @brief get the series inductance of the motor in micro henry
     *
     * @retval series inductance of the motor in micro henry
     */
    inline std::int32_t getMotorSeriesInductance(void) { return (series_inductance_); };

    /**
     * @brief add a adc reading in micro ampere
     */
    inline void addReading(std::int32_t ua)
    {
      current_reading_sum_ += ua;
      current_reading_count_++;
    }

    /**
     * @brief add a adc reading in micro ampere
     */
    inline std::int32_t getReading(void)
    {
      std::int32_t current_reading = current_reading_sum_ / current_reading_count_;
      current_reading_sum_ = current_reading_count_ = 0;
      return (current_reading);
    }

    /**
     * @brief Try to home the valve
     *
     * @retval true for successful competition of the homeing run. false for timeout
     */
    bool home(void);

    /**
     * @brief calculate the control loops
     *
     * @param Tms time since last call in milli seconds
     */
    void calculateControls(std::uint32_t Tms);

    /**
     * @brief standard constructor
     */
    ValveController(const gpio_num_t cur_pin, const gpio_num_t fwd_pin, const gpio_num_t bwd_pin);

  protected:
    // ESP Logging tag
    const char *TAG;
    // PWM frequency in Hz
    static const std::int32_t PWM_FREQUENCY;
    // ADC conversions per second
    static const std::uint32_t ADC_CONV_PER_SEC;
    // ADC conversions per pin
    static const std::uint32_t ADC_CONV_PER_PIN;
    // ADC Pin attenuation
    static const adc_atten_t ADC_ATTENUATION;
    // Conversion factor from mV to uA
    static const std::int32_t MV_TO_UA;

    // handle to adc instance
    static adc_continuous_handle_t adc_handle_ = nullptr;
    // config of the adc unit
    static adc_continuous_handle_cfg_t adc_config_;
    // adc calibration config
    static adc_cali_line_fitting_config_t cali_config_;
    // adc calibration handle
    static adc_cali_handle_t adc_cali_handle_ = nullptr;
    // adc callback configuration
    static adc_continuous_evt_cbs_t adc_callbacks_;
    // pwm timer configuration
    static ledc_timer_config_t ledc_timer_config_;

        // adc input gpio for current measurement
    const gpio_num_t ADC_PIN;
    // pwm output gpio for forward rotation.
    const gpio_num_t FWD_PIN;
    // pwm output gpio for backwards rotation
    const gpio_num_t BWD_PIN;

    // config of the adcs dig unit
    adc_continuous_config_t adc_dig_cfg_;
    // adc channel pattern config
    adc_digi_pattern_config_t adc_pattern_;

    // pwm channel for forward rotation
    ledc_channel_config_t ledc_channel_fwd_;
    // pwm channel for backward rotation
    ledc_channel_config_t ledc_channel_bwd_;

    // current measurement in uA
    std::int32_t actual_current_;
    // set current in uA
    std::int32_t set_current_;
    // current offset in uA
    std::int32_t offset_current_;
    // output voltage in uV
    std::int32_t output_voltage_;
    // offset samples needed just after boot
    bool offset_needed_;
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
    // Motor series resistance mOhm
    std::int32_t series_resistance_;
    // Motor series inductance uH
    std::int32_t series_inductance_;
    // Pi Controller for motor current
    control::pi ctrl_current_;
    // Pi Controller for motor speed
    control::pi ctrl_speed_;
    // Pi Controller for motor position
    control::pi ctrl_position_;

    /**
     * @brief setup the adc in general
     *
     * This function does the basic init of the ADC and needs to be called just once
     */
    static void initADC(void);

    /**
     * @brief setup the adc for this valve instance and start measuring
     */
    void startADC(void);

    /**
     * @brief stop the adc for this instance
     */
    void stopADC(void);

    /**
     * @brief setup the pwm in general
     *
     * This function does the basic init of the LEDC PWM unit and needs to be called just once
     */
    static void initPWM(void);

    /**
     * @brief setup the PWM Channels for this valve instance
     */
    void startPWM(void);

    /**
     * @brief Stop the PWM for this valve instance
     */
    void stopPWM(void);
  };

  /**
   * @brief initialize hardware components
   */
  extern void Init(void);
} /* namespace hardware */

#endif /* _HARDWARE_LAYER_H_ */