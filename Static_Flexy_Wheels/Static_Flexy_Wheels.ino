/*
  300007_Flexy_Wheels
  Created by Jason Swee, April 05, 2018.
  Description: To read the flex sensor and transmit in modes
*/
#include "Arduino.h"
#include <SoftwareSerial.h>

#define BLUETOOTH_SPEED 9600



#define RESIST_PIN A6
#define RESIST_RANGE 4 //Indicates the + - buffer error between resistance range
#define MODE_0 308 //Indicates the resistance value to trigger mode 1
#define MODE_1 295 //Indicates the resistance value to trigger mode 2


//Needed for Printing time
unsigned long last_print_time = 0;

SoftwareSerial BTSerial(10, 11); // RX, TX

int resistance = 0;
int inflate_mode = 0;
//int LED_pin = 3; //obsolete variable
int ledpin = 13;
int state =0;
void setup() 
{
  Serial.begin(9600);
  pinMode(ledpin,OUTPUT);
  BTSerial.begin(BLUETOOTH_SPEED);
  pinMode(RESIST_PIN, INPUT);
}
 
void loop() 
{ 
  check_resistance();
  //analogWrite(LED_pin,2); //Obsolete code
  bluetooth_transmit(inflate_mode);
  
  print_status(500);
}
void check_resistance()
{
  resistance = analogRead(RESIST_PIN);
  
  if (resistance >=(MODE_0-RESIST_RANGE) && resistance <= (MODE_0+RESIST_RANGE) || resistance > MODE_0 ) inflate_mode = 0;
  else if (resistance >=(MODE_1-RESIST_RANGE)&& resistance <=(MODE_1+RESIST_RANGE))inflate_mode = 1;
}
void bluetooth_transmit(int state)
{
  BTSerial.write(state);
}
void print_status(int print_every_x_millis)
{
  if(millis()- last_print_time >= print_every_x_millis)
  {
    //Serial.print("Transmitted Value: ");
    //Serial.print(1);
    //Serial.println(" ");
  
    Serial.print("Resistance Value: ");
    Serial.print(resistance);
    Serial.println(" ");

    Serial.print("Transmitted Value: ");
    Serial.print(inflate_mode);
    Serial.println(" ");
    
    last_print_time = millis();
    Serial.print("      Time:");
    Serial.println(last_print_time/1000);
  }
}

