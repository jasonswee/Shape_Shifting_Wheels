/*
  300007_RC_Shape_Shifting_Car for Controls
  Created by Jason Swee, April 05, 2018.
*/

#include "Arduino.h"
#define channel1_pin 1
#define channel2_pin 2
#define max_speed 240 
#define E1 5 //M1 speed control
#define M1 4 //M1 Directional control
#define E2 6 //M2 speed control
#define M2 7 //M2 Directional control

volatile int pwm_value1 = 0;
volatile int prev_time1 = 0;
volatile int pwm_value2 = 0;
volatile int prev_time2 = 0;

int c1 =0;//Left right
int c2 =0;//Left right

int power =0;
int left_power=0;
int right_power=0;


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
  remap();
  translator();
}
void remap()
{
  c1 = map(pwm_value1,1100,1900,100,0);//Left Right
  c2 = map(pwm_value2,1050,1900,100,-100);//Forward Reverse
}
void translator()
{
  power = max_speed * abs(c2)/100;

  if(c1<=51) 
  {
    
    c1 = map(c1,0,50,0,100);
  }
  else c1 = 
  left_power = power * c1 / 100;
  right_power = power - left_power;
  
  if(c2>0) //Forward
  {
    digitalWrite(M1,HIGH);
    digitalWrite(M2,LOW);
  }
  else
  {
    digitalWrite(M1,LOW);
    digitalWrite(M2,HIGH);
  }

  analogWrite(E1,left_power);
  analogWrite(E2,right_power);
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
    Serial.print("C1 PWM(LR): ");
    Serial.print(pwm_value1);

    Serial.print("  Calibrated 1: ");
    Serial.print(c1);

    Serial.print("     C2 PWM(LR): ");
    Serial.print(pwm_value2);

    Serial.print("  Calibrated 2: ");
    Serial.print(c2);

    Serial.print("  Power left: ");
    Serial.print(left_power);

    Serial.print("  Power right: ");
    Serial.print(right_power);
    
    last_print_time = millis();
    Serial.print("      Time:");
    Serial.println(last_print_time/1000);
  }
}
