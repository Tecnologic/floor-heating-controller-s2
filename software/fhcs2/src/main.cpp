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

#include <stdio.h>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hardware_layer.h"
#include "driver/gpio.h"

const char *TAG = "fhcs2";

extern "C" void app_main()
{
    hardware::Init();

    while (1)
    {
        static std::uint32_t counter = 0;
        static bool led = false;
        constexpr hardware::valve_channels_e chan = hardware::VALVE_CHAN_3;

        hardware::SetBoardLed(led);
        printf(">pos:%ld\n>speed:%ld\n>current:%ld\n>voltage:%ld\n",
               hardware::valve_controller[chan].getPosition(),
               hardware::valve_controller[chan].getSpeed(),
               hardware::valve_controller[chan].getCurrent(),
               hardware::valve_controller[chan].getVoltage());
        vTaskDelay(10 / portTICK_PERIOD_MS);
        counter++;

        if ((counter % 300) == 0)
        {
            hardware::valve_controller[chan].setSetCurrent(10000);
            led = true;
        }

        if ((counter % 600) == 0)
        {
            hardware::valve_controller[chan].setSetCurrent(-10000);
            led = false;
        }
    }
}