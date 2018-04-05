/*
  300007_RC_Shape_Shifting_Car
  Created by Jason Swee, April 05, 2018.
*/

#include "Arduino.h"
#define channel1_pin 1
#define channel2_pin 2

volatile int pwm_value1 = 0;
volatile int prev_time1 = 0;
volatile int pwm_value2 = 0;
volatile int prev_time2 = 0;

unsigned long last_print_time = 0;

void setup() 
{
  Serial.begin(9600);
  // when pin D2 goes high, call the rising function
  pinMode(channel1_pin, INPUT);
  pinMode(channel2_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(channel1_pin), rising1, RISING);
  attachInterrupt(digitalPinToInterrupt(channel2_pin), rising2, RISING);
}
 
void loop() 
{ 
  print_status(1000); 
}
 
void rising1() 
{
  attachInterrupt(digitalPinToInterrupt(channel1_pin), falling1, FALLING);
  prev_time1 = micros();
}
void rising2() 
{
  attachInterrupt(digitalPinToInterrupt(channel2_pin), falling2, FALLING);
  prev_time2 = micros();
}
void falling1() 
{
  attachInterrupt(digitalPinToInterrupt(channel1_pin), rising1, RISING);
  pwm_value1 = micros()-prev_time1;
}
void falling2() 
{
  attachInterrupt(digitalPinToInterrupt(channel2_pin), rising2, RISING);
  pwm_value2 = micros()-prev_time2;
}
void print_status(int print_every_x_millis)
{
  if(millis()- last_print_time >= print_every_x_millis)
  {
    Serial.print("PWM value1: ");
    Serial.print(pwm_value1);

    Serial.print("     PWM value2: ");
    Serial.print(pwm_value2);
    
    last_print_time = millis();
    Serial.print("      Time:");
    Serial.println(last_print_time/1000);
  }
}
