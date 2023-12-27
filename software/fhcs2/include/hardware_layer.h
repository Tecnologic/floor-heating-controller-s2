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

  enum bdc_channels_e
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
     * @brief set the current position setpoint of the valve
     *
     * @param position new position setpoint
     */
    void setPosition(const std::int32_t position);

    /**
     * @brief get the current position setpoint
     * @retval current position setpoint
     */
    std::int32_t getSetPosition(void);

    /**
     * @brief get the actual position of the valve
     *
     * @note during motion this value will be different from the
     *       position setpoint, because the drive will not jump to
     *       the next position.
     *
     * @retval actual position of the valve
     */
    std::int32_t getPosition(void);

    /**
     * @brief get the actual speed setpoint
     *
     * @retval actual set speed, output of position control loop
     */
    std::int32_t getSetSpeed(void);

    /**
     * @brief get the actual speed reference value
     *
     * @retval actual reference speed
     */
    std::int32_t getRefSpeed(void);

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
    std::int32_t getSpeed(void);

    /**
     * @brief get the speed ramp
     *
     * @retval actual speed ramp
     */
    std::int32_t getAcceleration(void);

    /**
     * @brief set the speed ramp
     *
     * @param ramp speed ramp
     */
    void setAcceleration(const std::int32_t ramp);

    /**
     * @brief set the current limit for homing
     *
     * @param current new homing current limit
     */
    void setHomingCurrent(const std::int32_t current);

    /**
     * @brief set the direction for homing
     *
     * @param forward true for forward rotaton and false for backwards rotation
     */
    void setHomingDirection(const bool forward);

    /**
     * @brief set the timeout for a homing try
     *
     * @param timeout homing timeout in ms
     */
    void setHomingTimeout(const std::uint32_t timeout);

    /**
     * @brief set homing speed
     *
     * @param speed new homing speed
     */
    void setHomingSpeed(const std::int32_t speed);

    /**
     * @brief get the current limit for homing
     *
     * @retval actual homing current limit
     */
    std::int32_t getHomingCurrent(void);

    /**
     * @brief get the direction for homing
     *
     * @retval true for forward rotaton and false for backwards rotation
     */
    bool getHomingDirection(void);

    /**
     * @brief get homing speed
     *
     * @retval actual homing speed
     */
    std::int32_t getHomingSpeed(void);

    /**
     * @brief get the timeout for a homing try
     *
     * @retval homing timeout in ms
     */
    std::uint32_t getHomingTimeout(void);

    /**
     * @brief Try to home the valve
     *
     * @retval true for successful competition of the homeing run. false for timeout
     */
    bool home(void);

    /**
     * @brief add a adc reading in micro ampere
     */
    void addReading(std::int32_t ua)
    {
      current_reading_sum_ += ua;
      current_reating_count_++;
    }

    /**
     * @brief calculate the control loops
     */
    void calculateControls(void)
    {
    }

  protected:
    // current measurement in uA
    std::int32_t actual_current_;
    // set current in uA
    std::int32_t set_current_;
    // current offset in uA
    std::int32_t offset_current_;
    // output voltage in mV
    std::int32_t output_voltage_;
    // offset samples
    std::uint32_t offset_samples_;
    // actual position
    std::int32_t actual_position_;
    // set position
    std::int32_t set_position_;
    // set speed
    std::int32_t set_speed_;
    // reference speed
    std::int32_t ref_speed_;
    // actuall speed
    std::int32_t actual_speed_;
    // acceleration
    std::int32_t acceleration_;
    // homing current
    std::int32_t homing_current_;
    // homing speed
    std::int32_t homing_speed_;
    // homing direction
    std::int32_t homing_direction_;
    // homing timeout
    std::uint32_t homing_timeout_;

    // current readings sum
    std::int32_t current_reading_sum_;
    // current reading count
    std::uint32_t current_reating_count_;
  };

  /**
   * @brief initialize hardware components
   */
  extern void Init(void);

  // valve controller instances
  extern std::array<ValveController, VALVE_CHAN_MAX> valve_controller;

} /* namespace hardware */

#endif /* _HARDWARE_LAYER_H_ */