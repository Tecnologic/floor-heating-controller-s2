#include <Arduino.h>
#include "bdc_control.h"
#include "filesystem.h"

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

extern void setupWifi();

void setup()
{
  Serial.begin(112500);
  delay(10000);
  Serial.println("Setup BDC!");
  bdc::init();
  Serial.println("Setup BDC complete!");

  Serial.println("Setup FS!");
  filesystem::init();
  Serial.println("Setup FS complete!");
}

void loop()
{
  std::uint32_t i = 0;
  std::int32_t volt = 5000;
  for (auto motor : motors)
  {
    i++;
    Serial.printf("Motor %d Current %d uA, Set Voltage %d mV\n", i, motor->getCurrent(), volt);
    motor->setVoltage(volt);
  }
  volt = -volt;
  delay(5000);
}
