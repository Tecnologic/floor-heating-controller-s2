#ifndef _HARDWARE_LAYER_H_
#define _HARDWARE_LAYER_H_

#include <cstdint>
#include <array>

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

  enum valve_channels_e
  {
    VALVE_CHAN_1,
    VALVE_CHAN_2,
    VALVE_CHAN_3,
    VALVE_CHAN_4,
    VALVE_CHAN_5,
    VALVE_CHAN_6,
    VALVE_CHAN_7,
    VALVE_CHAN_8,
    VALVE_CHAN_MAX,
  };

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
     * @brief set the actual voltage supplied to the motor
     *
     * @param u_volt voltage output of the bridge to the motor in uV
     */
    inline void setVoltage(const std::uint32_t u_volt) { output_voltage_ = u_volt; };

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
     * @brief set the output voltage for the motor
     */
    void updateVoltage(const std::uint8_t channel);

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

    ValveController() : // current measurement in uA
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
                        // actuall speed
                        actual_speed_(0),
                        // acceleration
                        acceleration_(1000000),
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
                        series_inductance_(16300){};

  protected:
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
    std::uint32_t series_resistance_;
    // Motor series inductance uH
    std::uint32_t series_inductance_;
  };

  /**
   * @brief initialize hardware components
   */
  extern void Init(void);

  // valve controller instances
  extern std::array<ValveController, VALVE_CHAN_MAX> valve_controller;

} /* namespace hardware */

#endif /* _HARDWARE_LAYER_H_ */