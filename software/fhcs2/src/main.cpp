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
        hardware::SetBoardLed(!hardware::GetBoardLed());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}