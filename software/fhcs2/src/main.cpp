#include <cstdint>
#include <Arduino.h>
// #include "bdc_control.h"
// #include "filesystem.h"
// #include "wifimanager.h"

// constexpr std::uint32_t NO_OF_MOTORS = 1UL;

// using bdc = BdcSensorlessPositionControl<NO_OF_MOTORS>;
// using bdc_array = std::array<bdc *, NO_OF_MOTORS>;

// bdc motor_1(9, 10, 1);
// // bdc motor_2(11, 12, 2);
// // bdc motor_3(13, 14, 3);
// // bdc motor_4(16, 17, 4);
// // bdc motor_5(18, 19, 5);
// // bdc motor_6(33, 34, 6);
// // bdc motor_7(35, 16, 7);
// // bdc motor_8(37, 38, 8);

// bdc_array motors = {&motor_1,
//                     /*&motor_2,
//                     &motor_3,
//                     &motor_4,
//                     &motor_5,
//                     &motor_6,
//                     &motor_7,
//                     &motor_8*/};

void wait_for_seconds(std::uint32_t sec)
{
  Serial.print("Waiting for ");
  Serial.print(sec);
  Serial.print("sec   ");
  for (std::uint8_t i = 0; i < 10; i++)
  {
    Serial.print(".");
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
  }
  Serial.println("");
}

void setup()
{
    // PWM frequency
  static constexpr std::uint32_t PWM_FREQUENCY = 16000;
  // PWM bit width
  static constexpr std::uint32_t PWM_BIT_WIDTH = 12;
  uint8_t pwm_pin = 11;
  uint8_t dir_pin = 12;
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(112500);
  wait_for_seconds(20);

    Serial.print(" PWM Pin ");
    Serial.print(pwm_pin);
    // Setup timer and attach timer to a led pin
    ledcAttach(pwm_pin, PWM_FREQUENCY, PWM_BIT_WIDTH);
    Serial.print(" Dir Pin ");
    Serial.println(dir_pin);
    pinMode(dir_pin, OUTPUT);

  // Serial.println("Setup BDC!");
  // bdc::init();
  // Serial.println("Setup BDC complete!");

  // Serial.println("Setup FS!");
  // filesystem::init();
  // Serial.println("Setup FS complete!");

  // Serial.println("Setup Wifi!");
  // wifimanager::init();
  // Serial.println("Setup Wifi complete!");
}

void loop()
{
  static std::uint32_t seconds = 0;
  // std::uint32_t i = 0;
  // std::int32_t volt = 5000;
  // for (auto motor : motors)
  // {
  //   i++;
  //   Serial.printf("Motor %d Current %d uA, Set Voltage %d mV\n", i, motor->getCurrent(), volt);
  //   motor->setVoltage(volt);
  // }
  // volt = -volt;
  // wifimanager::handle();
  // seconds++;
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  seconds++;
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  Serial.print("running for ");
  Serial.print(seconds);
  Serial.println("sec   ");
}
