#include "esphome/core/log.h"
#include "valve.h"

namespace esphome {
namespace valve {

static const char *TAG = "valve.component";
static const gpio_num_t LED_GPIO = static_cast<gpio_num_t>(15);

TickType_t last_tick = 0;
bool s_led_state = 0;

void Valve::setup() {
    ESP_LOGI(TAG, "Valve Setup GPIO %d as output", LED_GPIO);


    gpio_reset_pin(LED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    last_tick = xTaskGetTickCount();
}

void Valve::loop() {
    
    TickType_t current_tick = xTaskGetTickCount();
    TickType_t diff = pdMS_TO_TICKS(1000);
    if((current_tick - last_tick) > diff)
    {
        current_tick = xTaskGetTickCount();
        last_tick = current_tick;
        /* Set the GPIO level according to the state (LOW or HIGH)*/
        ESP_LOGI(TAG, "Turning the LED %s! at Tick %d, Last %d, Diff %d\n", s_led_state == true ? "ON" : "OFF", current_tick, last_tick, diff);
        gpio_set_level(LED_GPIO, s_led_state);
        /* Toggle the LED state */
        s_led_state = !s_led_state;
    }
}

void Valve::dump_config(){
    ESP_LOGCONFIG(TAG, "Valve");
}


}  // namespace valve
}  // namespace esphome