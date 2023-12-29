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

    hardware::valve_controller[hardware::VALVE_CHAN_2].setVoltage(3000000);

    while (1)
    {
        static bool led = false;
        led = !led;
        hardware::SetBoardLed(led);
        printf("pos:%ld\n speed:%ld\n current:%ld\nled:%d\n",
               hardware::valve_controller[hardware::VALVE_CHAN_2].getPosition(),
               hardware::valve_controller[hardware::VALVE_CHAN_2].getSpeed(),
               hardware::valve_controller[hardware::VALVE_CHAN_2].getCurrent(),
               hardware::GetBoardLed());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}