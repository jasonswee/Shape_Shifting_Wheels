/*
  300007_Shape_Shifting_Wheels
  Created by Jason Swee, April 05, 2018.
*/

#include "Arduino.h"
#include <Encoder.h> //Encoder library

//Set the encoder pins below
#define ENCODER_PINA 2
#define ENCODER_PINB 4
#define POS_TO_VEL_CONST 1

Encoder myEnc(ENCODER_PINA, ENCODER_PINB); //Set pins for encoder

unsigned long old_time = 0;
unsigned long new_time = 0;
unsigned long velocity = 0;
long encoded_position = 0;

int E2 = 6; //M2 speed control
int M2 = 7; //M2 Directional control
int count = 0;
int desired_spd = 0;

unsigned long last_print_time = 0;
unsigned long last_poll_time = 0;
int debounce_delay = 10;
  
//The setup function is called once at startup of the sketch
void setup()
{
// Add your initialization code here
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");

  //pinMode(ENCODER_PINA, INPUT);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), pin_ISR, CHANGE);
  
  pinMode(E2, OUTPUT); 
  pinMode(M2, OUTPUT); 
  digitalWrite(M2,LOW);
  analogWrite(E2,100);

  last_poll_time = millis();
  last_print_time = millis();
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here

  //Simulate changing speed
  simulate_motor_spd(true); //Argument is constant speed for true or varying speed for false
  
  new_time = millis();

  //Print every 0.1 second, argument milliseconds of frequency
  poll_motor_spd(100);

  //Print every 1 second, argument milliseconds of frequency
  print_status(1000); 

  
  
}
void poll_motor_spd(int poll_every_x_millis)
{
  if(millis()-last_poll_time >= poll_every_x_millis)
  {
    encoded_position = myEnc.read();
    velocity = encoded_position*POS_TO_VEL_CONST/(poll_every_x_millis);
    last_poll_time = millis();
    myEnc.write(0);
    delay(debounce_delay);
  }
}
void simulate_motor_spd(boolean constant_speed_check)
{
  if(constant_speed_check == true) analogWrite(E2,150);
  else
  {
    analogWrite(E2,desired_spd);//desired_spd);
    count++;
    if(count>=2000)desired_spd+=10;
    if(desired_spd>=200)desired_spd=0;
  }
}
void print_status(int print_every_x_millis)
{
  if(millis()- last_print_time >= print_every_x_millis)
  {
    Serial.print("Velocity:");
    Serial.print(velocity);
    
    Serial.print("      Encoder:");
    Serial.print(encoded_position);
    
    last_print_time = millis();
    Serial.print("      Time:");
    Serial.println(last_print_time/1000);
  }
}

