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

        Floor Heating Controller S2 is free software: you can redistribute it
   and/or modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation, either version 3 of the License,
   or (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware_layer.h"
#include "string.h"
#include <cstdint>
#include <stdio.h>

const char *TAG = "fhcs2";

extern "C" void app_main()
{

   vTaskDelay(10000 / portTICK_PERIOD_MS);
   hardware::Init();

   while (1)
   {
      static std::uint32_t counter = 0;
      static bool led = false;
      constexpr hardware::ValveController::channels_e chan =
          hardware::ValveController::CHANNEL_3;

      hardware::SetBoardLed(led);
      vTaskDelay(10 / portTICK_PERIOD_MS);
      counter++;
      static std::int32_t current = 0;

      hardware::valves[chan].move(1);

      if ((counter % 300) == 0)
      {
         led = true;
         current = 20000;
      }
      else if ((counter % 600) == 0)
      {
         led = false;
         current = 0;
      }
      printf(">cur:%ld\n>v:%ld\n>set:%ld\n", hardware::valves[chan].getCurrent(), hardware::valves[chan].getVoltage(), current);
      hardware::valves[chan].setSetCurrent(current);
   }
}