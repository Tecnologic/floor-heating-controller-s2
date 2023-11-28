#include <Arduino.h>
#include "bdc_control.h"
#include "filesystem.h"
#include "wifimanager.h"
#include "esp_log.h"


constexpr std::uint32_t NO_OF_MOTORS = 8UL;

using bdc = BdcSensorlessPositionControl<NO_OF_MOTORS>;
using bdc_array = std::array<bdc *, NO_OF_MOTORS>;

bdc motor_1(9, 10, 1);
bdc motor_2(11, 12, 2);
bdc motor_3(13, 14, 3);
bdc motor_4(16, 17, 4);
bdc motor_5(18, 19, 5);
bdc motor_6(33, 34, 6);
bdc motor_7(35, 16, 7);
bdc motor_8(37, 38, 8);

bdc_array motors = {&motor_1,
                    &motor_2,
                    &motor_3,
                    &motor_4,
                    &motor_5,
                    &motor_6,
                    &motor_7,
                    &motor_8};

void wait_for_seconds(std::uint32_t sec)
{
  USBSerial.print("Waiting for ");
  USBSerial.print(sec);
  USBSerial.print("sec   ");
  for (std::uint8_t i = 0; i < 10; i++)
  {
    USBSerial.print(".");
    digitalWrite(LED_BUILTIN, LOW);    
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);    
    delay(500);
  }
  USBSerial.println("");
}
 

void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  USBSerial.begin(112500);
  USBSerial.setDebugOutput(true);
  esp_log_level_set("*", ESP_LOG_VERBOSE);        // set all components to ERROR level
  wait_for_seconds(20);

  USBSerial.println("Setup BDC!");
  // bdc::init();
  USBSerial.println("Setup BDC complete!");

  wait_for_seconds(20);
  USBSerial.println("Setup FS!");
  // filesystem::init();
  USBSerial.println("Setup FS complete!");

  wait_for_seconds(20);
  USBSerial.println("Setup Wifi!");
  // wifimanager::init();
  USBSerial.println("Setup Wifi complete!");
}

void loop()
{
  static std::uint32_t seconds = 0;
  // std::uint32_t i = 0;
  // std::int32_t volt = 5000;
  // for (auto motor : motors)
  // {
  //   i++;
  //   USBSerial.printf("Motor %d Current %d uA, Set Voltage %d mV\n", i, motor->getCurrent(), volt);
  //   motor->setVoltage(volt);
  // }
  // volt = -volt;
  // wifimanager::handle();
  seconds++;
  digitalWrite(LED_BUILTIN, LOW);    
  delay(1000);
  seconds++;
  digitalWrite(LED_BUILTIN, HIGH);    
  delay(1000);
  USBSerial.print("running for ");
  USBSerial.print(seconds);
  USBSerial.println("sec   ");
}
