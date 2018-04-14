/*
  300007_Flexy_Wheels
  Created by Jason Swee, April 05, 2018.
  Description: To read the flex sensor and transmit in modes
*/
#include "Arduino.h"
#include <SoftwareSerial.h>

#define BLUETOOTH_SPEED 9600

#define RESIST_PIN A2
#define RESIST_RANGE 20 //Indicates the + - buffer error between resistance range
#define MODE_1 200 //Indicates the resistance value to trigger mode 1
#define MODE_2 200 //Indicates the resistance value to trigger mode 2
#define MODE_3 300 //Indicates the resistance value to trigger mode 2

//Needed for Printing time
unsigned long last_print_time = 0;

SoftwareSerial BTSerial(10, 11); // RX, TX

int resistance = 0;
int mode = 0;

void setup() 
{
  Serial.begin(9600);
  BTSerial.begin(BLUETOOTH_SPEED);
  pinMode(RESIST_PIN, INPUT);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.println("Starting...");
}
 
void loop() 
{ 
  
  check_resistance();
  bluetooth_transmit(char(mode));

  print_status(1000); //Print time
}
void check_resistance()
{
  resistance = analogRead(RESIST_PIN);
  Serial.print("Resistance Value: ");
  Serial.print(resistance);
  if (resistance<(MODE_1-RESIST_RANGE))mode = 0;
  else if (resistance>(MODE_1-RESIST_RANGE)&& resistance<(MODE_1+RESIST_RANGE))mode = 1;
  else if (resistance>(MODE_2-RESIST_RANGE)&& resistance<(MODE_2+RESIST_RANGE))mode = 2;
  else if (resistance>(MODE_3-RESIST_RANGE)&& resistance<(MODE_3+RESIST_RANGE))mode = 3;
  else mode = 4;
}
void bluetooth_transmit(char state)
{
  BTSerial.write(state);
  Serial.print("Transmitted Value: ");
  Serial.print(state);
}
void print_status(int print_every_x_millis)
{
  if(millis()- last_print_time >= print_every_x_millis)
  {
    last_print_time = millis();
    Serial.print("      Time:");
    Serial.println(last_print_time/1000);
  }
}

