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
#ifndef INC_CONTROLLER_HPP_
#define INC_CONTROLLER_HPP_

#include <cstdint>
#include <cmath>
#include <climits>

/**
 * @namespace controller classes
 */
namespace control
{
	/**
	 * pi controller with anti windup
	 */
	class pi
	{
	private:
		///< integral error sum
		std::int32_t error_sum;
		///< unlimited controller output
		std::int32_t output_unlimited;
		///< limited controller output
		std::int32_t output;
		///< proportional controller gain
		std::int32_t kp;
		///< integral action time in us
		std::int32_t tn;

	public:
		///< positive output limit (not necessarily positive)
		std::int32_t positive_limit;
		///< negative output limit (not necessarily negative)
		std::int32_t negative_limit;

		/**
		 * @brief Pi controller constructor with all essential parameters.
		 *
		 * @param new_kp                proportional gain.
		 * @param new_tn                integral action time in us.
		 * @param new_positive_limit    positive output limit.
		 * @param new_negative_limit    negative output limit.
		 */
		pi(const std::int32_t kp, const std::int32_t tn,
			 const std::int32_t positive_limit, const std::int32_t negative_limit);
		/**
		 * @brief calculate regulator equation with feed forward and anti windup.
		 *
		 * @param setpoint			setpoint
		 * @param act						actual
		 * @param feed_forward  feed forward control input.
		 * @param Ts						sampling time in us
		 * @retval controller output
		 */
		std::int32_t Calculate(const std::int32_t setpoint, const std::int32_t actual, const std::int32_t feed_forward, const std::int32_t Ts);

		/**
		 * @brief set controller dynamic parameters.
		 *
		 * @param new_kp				proportional gain.
		 * @param new_tn				integral action time.
		 */
		constexpr inline void SetParameters(const std::int32_t new_kp, const std::int32_t new_tn, const std::int32_t ts)
		{
			kp = new_kp;
			tn = new_tn;
		};

		///< @brief Reset controller and integral part to 0
		constexpr inline void Reset(void)
		{
			error_sum = 0;
			output_unlimited = 0;
			output = 0;
		};
	};

} /* namespace control */

#endif /* INC_CONTROLLER_HPP_ */
