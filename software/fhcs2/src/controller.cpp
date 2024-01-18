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
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include "controller.h"

/**
 * @namespace controller classes
 */
namespace control
{

	/**
	 * @brief Pi controller constructor with all essential parameters.
	 *
	 * @param new_kp				proportional gain.
	 * @param new_tn				integral action time.
	 * @param new_positive_limit	positive output limit.
	 * @param new_negative_limit	negative output limit.
	 */
	pi::pi(const std::int32_t new_kp, const std::int32_t new_tn,
		   const std::int32_t new_positive_limit,
		   const std::int32_t new_negative_limit) : error_sum(0),
													output_unlimited(0),
													output(0),
													kp(new_kp),
													tn(new_tn),
													positive_limit(new_positive_limit),
													negative_limit(new_negative_limit)
	{
	}

	/**
	 * @brief calculate regulator equation with feed forward and anti windup.
	 *
	 * @param error			control loop error.
	 * @param feed_forward 	feed forward control input.
	 * @retval controller output
	 */
	std::int32_t pi::Calculate(const std::int32_t setpoint, const std::int32_t actual, const std::int32_t feed_forward, const std::int32_t Ts)
	{
		std::int32_t error = setpoint - actual;

		output_unlimited = error_sum + (error * kp) / 1000 + feed_forward; // regulator equation

		if (output_unlimited > positive_limit) // upper saturation
		{
			output = positive_limit;
		}
		else if (output_unlimited < negative_limit) // lower saturation
		{
			output = negative_limit;
		}
		else // normal operation
		{
			output = output_unlimited;
		}

		if (tn > 0)
		{
			// anti windup strategy is only to limit if
			// integration of the error takes the output
			// more over the limit.
			// if not we are free to integrate
			if ((output == output_unlimited) || ((error * output_unlimited) <= 0))
			{
				error_sum += (error * kp * Ts) / (tn * 1000);
			}
		}
		return output;
	}

} /* namespace control */
