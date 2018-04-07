/*
  300007_RC_Shape_Shifting_Wheels for Controls
  Created by Jason Swee, April 05, 2018.
*/

#include "Arduino.h"
#define C1_PIN 1
#define MAX_POWER 100 
#define E1 5 //M1 speed control     //PUMP
#define M1 4 //M1 Directional control
#define E2 6 //M2 speed control     //Valve
#define M2 7 //M2 Directional control
#define MAX_INFLATE_TIME 3 //Seconds
#define DEFLATE_COOLDOWN 5

volatile int pwm_value1 = 0;
volatile int prev_time1 = 0;

int c1 =0;//Inflate deflate

unsigned long new_time = 0;
unsigned long last_print_time = 0;
unsigned long last_inflate_time = 0;
unsigned long last_deflate_time = 0;
boolean inflating = false;
boolean deflating = false;
boolean inflated_check = false;
boolean deflated_check = true;

int state = 0;
int power = 0;
int lapsed_inflating_time = 0;
int lapsed_deflating_time = 0;
int required_deflation_time = 0;
int new_time_in_sec = 0;


void state_excution()
{
  if(state == 0)//Normal
  {
    analogWrite(E1,0); //Pump off
    analogWrite(E2,0); //Valve closed
  }
  else if(state == 1)//Inflating
  {
    analogWrite(E1,power); //Pump off
    analogWrite(E2,200); //Valve open
  }
  else if(state == 2)//Inflated
  {
    analogWrite(E1,0); //Pump closed
    analogWrite(E2,0); //Valve closed
  }
  else if(state == 3)//Deflating
  {
    analogWrite(E1,0); //Pump off
    analogWrite(E2,200); //Valve open
  }
}

void remap()
{
  c1 = map(pwm_value1,1100,2000,100,-100);//Up down
}
void setup() 
{
  Serial.begin(9600);
  // when pin D2 goes high, call the rising function
  pinMode(C1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(C1_PIN), rising1, RISING);

  digitalWrite(M1,HIGH);
  digitalWrite(M2,HIGH);
}
 
void loop() 
{ 
  new_time = millis();
  new_time_in_sec = new_time/1000;
  print_status(1000); 
  remap();
  staging();
  state_excution();
}
void staging()
{
  int starting_power_threshold = 70;

  if(abs(c1)> starting_power_threshold)power = MAX_POWER * c1/100; //Power ranges -ve to +ve
  else power = 0;
  
  if(state == 0) //Lock valve 
  {
    if(power > 0)
    {
      state = 3;
      last_inflate_time = millis()/1000;
    }   
  }
  else if(state == 1) //Inflating
  {
    lapsed_inflating_time = millis()/1000 - last_inflate_time;
    if(lapsed_inflating_time >= MAX_INFLATE_TIME) 
    {
      last_deflate_time = millis()/1000;
      state = 2;
    }
  }
  else if(state == 2) //Inflated
  {
    if(power <0) 
    {
      state = 3;
      last_deflate_time = millis()/1000;
    }
    
  }
  else if(state == 3) //deflating
  {
    lapsed_deflating_time = millis()/1000 - last_deflate_time;
    if(lapsed_deflating_time >= DEFLATE_COOLDOWN) state =0;
  }
  
}
void rising1() 
{
  attachInterrupt(digitalPinToInterrupt(C1_PIN), falling1, FALLING);
  prev_time1 = micros();
}
void falling1() 
{
  attachInterrupt(digitalPinToInterrupt(C1_PIN), rising1, RISING);
  pwm_value1 = micros()-prev_time1;
}
void print_status(int print_every_x_millis)
{
  if(millis()- last_print_time >= print_every_x_millis)
  {
    Serial.print("Channel 1 PWM value(LR): ");
    Serial.print(pwm_value1);

    Serial.print("  Calibrate 1: ");
    Serial.print(c1);

    Serial.print("  State: ");
    Serial.print(state);
    if(state == 0) Serial.print("  Normal  ");
    else if(state == 1) Serial.print("  Inflating  ");
    else if(state == 2) Serial.print("  Inflated  ");
    else if(state == 3) Serial.print("  deflating  ");
    last_print_time = millis();
    Serial.print("      Time:");
    Serial.println(last_print_time/1000);
  }
}

