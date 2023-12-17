#include <stdio.h>
#include <cstdint>
#include "string.h"
#include "esp_err.h"
#include "esp_log.h"
#include "bdc_control.h"
#include "driver/gpio.h"

const char* TAG = "fhcs2";

constexpr std::uint32_t NO_OF_MOTORS = 2UL;

using bdc = BdcSensorlessPositionControl<NO_OF_MOTORS>;
using bdc_array = std::array<bdc *, NO_OF_MOTORS>;

bdc motor_1(GPIO_NUM_9, GPIO_NUM_10, GPIO_NUM_1);
bdc motor_2(GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_2);
// bdc motor_3(GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_3);
// bdc motor_4(GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_4);
// bdc motor_5(GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_5);
// bdc motor_6(GPIO_NUM_33, GPIO_NUM_34, GPIO_NUM_6);
// bdc motor_7(GPIO_NUM_35, GPIO_NUM_16, GPIO_NUM_7);
// bdc motor_8(GPIO_NUM_37, GPIO_NUM_38, GPIO_NUM_8);

bdc_array motors = {&motor_1,
                    &motor_2,
                    /*&motor_3,
                    &motor_4,
                    &motor_5,
                    &motor_6,
                    &motor_7,
                    &motor_8*/};

extern "C" void app_main() 
{
    std::uint32_t count = 0;
    ESP_LOGI(TAG, "Started Setup BDC");
    bdc::init();
    ESP_LOGI(TAG, "Setup BDC complete!");
    
    while(1)
    {
        static std::int32_t volt = 0;
        static bool invert = false;
        std::int32_t step = 50;

        ESP_LOGI(TAG, "Count: %lu", count++);
         
        for (auto motor : motors)
        {
            motor->setVoltage(volt);
        }

        if (invert)
        {
            volt -= step;
        }
        else
        {
            volt += step;
        }

        if (volt >= bdc::PWM_VOLTAGE)
        {
            invert = true;
        }

        if (volt <= -bdc::PWM_VOLTAGE)
        {
            invert = false;
        }

        gpio_set_level(bdc::LED_PIN, count % 2UL);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}