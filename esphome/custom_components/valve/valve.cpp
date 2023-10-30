#include "valve.h"
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
namespace esphome
{
    namespace valve
    {

        const char *TAG = "valve.component";

        void Valve::setup()
        {
        }

        void Valve::loop()
        {
        }

        void Valve::dump_config()
        {
            ESP_LOGCONFIG(TAG, "Valve");
        }

    } // namespace valve
} // namespace esphome