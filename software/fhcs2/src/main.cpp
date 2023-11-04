#include <Arduino.h>
#include "bdc_control.h"

std::array<BdcSensorlessPositionControl, 8> motors =
    {
        {.pwm_pin = 9, .dir_pin = 10, adc_pin = 1},
        {.pwm_pin = 11, .dir_pin = 12, adc_pin = 2},
        {.pwm_pin = 13, .dir_pin = 14, adc_pin = 3},
        {.pwm_pin = 16, .dir_pin = 17, adc_pin = 4},
        {.pwm_pin = 18, .dir_pin = 19, adc_pin = 5},
        {.pwm_pin = 33, .dir_pin = 34, adc_pin = 6},
        {.pwm_pin = 35, .dir_pin = 36, adc_pin = 7},
        {.pwm_pin = 37, .dir_pin = 38, adc_pin = 8},
};

void setup()
{
  Serial.begin(112500);
  delay(1000);
  BdcSensorlessPositionControl::init();
}

void loop()
{
  // Check if conversion is done and try to read data
  if (adc_coversion_done == true)
  {
    // Set ISR flag back to false
    adc_coversion_done = false;
    // Read data from ADC
    if (analogContinuousRead(&result, 0))
    {
      for (int i = 0; i < adc_pins_count; i++)
      {
        Serial.printf("\nADC PIN %d data:", result[i].pin);
        Serial.printf("  Avg raw value = %d", result[i].avg_read_raw);
        Serial.printf("  Avg millivolts value = %d", result[i].avg_read_mvolts);
      }

      // Delay for better readability of ADC data
      delay(1000);
    }
    else
    {
      Serial.println("Error occured during reading data. Set Core Debug Level to error or lower for more informations.");
    }
  }
}

// This configures the MCPWM peripheral to produce complimentary A & B PWM signals to drive upper & Lower
// MOSFETS in a Half bridge. dead time is required
// Two PWM channels, 180 degrees shifted are required for an interleaved synchronous buck charger project (48A battery charging from solar or 3 x server supplies
// in series).
// Four outputs (A&B for Phase 1, plus A&B for phase 2) are needed with 180 degree phase shift between them.
//!!!MY PROBLEM - HOW TO SET BOTH A&B Low to DISABLE both Half bridge FETs??
// Being able to set both low is needed for bridge drivers like UCC27211
// Is it a job for Fault module?  How!  Register operation?? Different Timer / generator / deadtime strategy OR add external logic and enable line
//(but this is a sophisticated MCPWM! - must be possible?!?

#include "arduino.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define GPIO_PWM0A_OUT 35  // High side FET PHASE1
#define GPIO_PWM0B_OUT 36  // Low Side FET PHASE1
#define GPIO_2PWM0A_OUT 39 // High side FET PHASE2
#define GPIO_2PWM0B_OUT 40 // Low Side FET PHASE2

#define GPIO_SCOPE_Trigger_Sig 38 // For testing - trigger scope to monitor signals at start up, PWM changes and disabling

int frequency = 60000;
float PWM = 15;

// float Rise_delay = 200-9f;

bool pwm0 = false;
bool pwm1 = false;
bool buckEnable = false;

//************************************************************
void MCPWM_SetUP()
{

  // we set up the MCPWM to create complimentary PWM with a set dead time on rising edges...

  mcpwm_group_set_resolution(MCPWM_UNIT_0, 160000000);
  mcpwm_timer_set_resolution(MCPWM_UNIT_0, MCPWM_TIMER_0, 160000000);
  mcpwm_timer_set_resolution(MCPWM_UNIT_0, MCPWM_TIMER_1, 160000000); // Found necessary to apply this to all timers

  // Config structure for timers....
  mcpwm_config_t pwm_config;
  pwm_config.frequency = frequency; // frequency = 1000Hz
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);

  delay(20);
  // To sync times and apply a phase shift, Enabling sync output of another timer by invoking mcpwm_set_timer_sync_output()
  // and selecting desired event to generate sync output from:-
  mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ); //'MCPWM_SWSYNC_SOURCE_TEZ' = the sync signal is generated when Timer0 counts to zero
  mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_TIMER0_SYNC, 0);       // Dont think this is needed?... sync source is from TIMER_0
  mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_TIMER0_SYNC, 500);     // timer 2 has 180 degree shift

  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 50, 50);
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 50, 50);

  // PWM starts operating using the set PWM Duty value...
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, PWM);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, PWM);

  delay(20);
  // mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  // vTaskDelay(pdMS_TO_TICKS(10));
  delay(20);
  // mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);   //gives a clean cycle start to PWM generator, no part duty waveforms

  delay(17);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT); // PWM generation commences following init commands
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT); // Phase 1 GPIOs

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_2PWM0A_OUT); // PWM generation commences following init commands
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_2PWM0B_OUT); // Phase 2 GPIOs

  // digitalWrite(GPIO_SCOPE_Trigger_Sig, HIGH);     //Toggle a pin..for scope diagnostic purposes
  buckEnable = 1;
}

//***************************************************************
void buck_Disable(void)
{

  // This is my problem!How to force BOTH A and B generator outputs Low when using MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE
  // Tried changing the dead time mode such that A and B can be independently controlled...
  // Tried just setting them low but it can extent high time of A or B - doesnt feel correct!
  // Is it a job for Fault module ?  Setting registers

  // pinMode(GPIO_PWM0A_OUT, OUTPUT);    // sets the digital pin 13 as output
  // pinMode(GPIO_PWM0B_OUT, OUTPUT);    // sets the digital pin 13 as output

  digitalWrite(GPIO_SCOPE_Trigger_Sig, HIGH); // sets the status monitor pin high
  digitalWrite(GPIO_PWM0A_OUT, LOW);          // sets the status monitor pin low
  digitalWrite(GPIO_PWM0B_OUT, LOW);          // sets the status monitor pin low

  digitalWrite(GPIO_SCOPE_Trigger_Sig, LOW); // sets the status monitor pin high

  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

  /* mcpwm_deadtime_enable(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_BYPASS_RED,0,0);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    //mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_GEN_B,0);
    //mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_GEN_A,0);
    buckEnable=0;
    */
}

//****************************************************************
void Set_PWM(float duty)
{
  if (duty > 4)
  {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, duty);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, duty);
  }
  else
  {
    buck_Disable();
  }
}

//**********************************************************************

void setup()
{
  pinMode(GPIO_SCOPE_Trigger_Sig, OUTPUT);   // sets the digital pin 13 as output
  digitalWrite(GPIO_SCOPE_Trigger_Sig, LOW); // sets the status monitor pin low
}

//**********************************************************************

void loop()
{

  delay(2000);
  MCPWM_SetUP();
  // digitalWrite(GPIO_SCOPE_Trigger_Sig, HIGH); // sets the status monitor pin low

  delay(2000);
  // digitalWrite(GPIO_SCOPE_Trigger_Sig, HIGH); // sets the status monitor pin high
  buck_Disable();

  delay(2000);
  // digitalWrite(GPIO_SCOPE_Trigger_Sig, LOW); // sets the status monitor pin high
  /*

    //digitalWrite(GPIO_SCOPE_Trigger_Sig, HIGH); // sets the status monitor pin low
    PWM=5;
    buck_Enable();

     for (int i = 5; i <= 95; i++) {
       PWM=i;
       Set_PWM(PWM);
       delay(10);
       }

   delay(3000);
  */
}