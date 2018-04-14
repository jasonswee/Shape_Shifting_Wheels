/*
  300007_Shape_Shifting_Wheels
  Created by Jason Swee, April 05, 2018.
*/

#include "Arduino.h"
#include <Encoder.h> //Encoder library
#include <SoftwareSerial.h>

//Set the Bluetooth pins below
SoftwareSerial BTSerial(10, 11); //RX,TX
#define BLUETOOTH_SPEED 9600

//Set the encoder pins below
#define ENCODER_PINA 2
#define ENCODER_PINB 4
#define POS_TO_VEL_CONST 1

//Set the Rain threshold
#define RAIN_PIN A2
#define RAIN_THRESHOLD 100

//Set pushbutton pin
#define PUSH_PIN 5

//Set RGB led 

Encoder myEnc(ENCODER_PINA, ENCODER_PINB); //Set pins for encoder


//Encoder reading function
unsigned long old_time = 0;
unsigned long new_time = 0;
unsigned long velocity = 0;
unsigned long encoded_position = 0;
unsigned long last_poll_time = 0;
int debounce_delay = 10;

//Simulated speed function
int E2 = 6; //M2 speed control
int M2 = 7; //M2 Directional control
int count = 0;
int desired_spd = 0;

//Pump function
int E1 = 4;//M1 speed control
int M1 = 5;//M1 Directional control

//Mode to for receiving BT
int mode = 0;

//Push button
boolean inflate_check = false;

//Rain module 
int rain_value = 0;

//Needed for printing time
unsigned long last_print_time = 0;

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

  //Rain
  pinMode(RAIN_PIN,OUTPUT);

  //Push
  pinMode(PUSH_PIN,OUTPUT);

  //Pump
  pinMode(E1, OUTPUT); 
  pinMode(M1, OUTPUT); 
  digitalWrite(M2,LOW);
  
  last_poll_time = millis();
  last_print_time = millis();

  Serial.begin(9600);
  BTSerial.begin(BLUETOOTH_SPEED);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.println("Starting...");
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here

  //Simulate changing speed
  //Comment off to switch off simulate speed
  simulate_motor_spd(true); //True to run constant speed or varying speed for false
  
  new_time = millis();
  
  //Capture received mode
  mode = bluetooth_receive();

  //Capture Rain and push button values
  rain_or_skidding();

  //Print every 0.1 second, argument milliseconds of frequency
  poll_motor_spd(100);

  //Print every 1 second, argument milliseconds of frequency
  print_status(1000); 

  control_pump(mode);

}
void rain_or_skidding()
{
  rain_value = analogRead(RAIN_PIN);

  if(rain_value > RAIN_THRESHOLD)inflate_check = true;
  if(digitalRead(PUSH_PIN)== HIGH)inflate_check = true;
  delay(100); //Software debounce for push button
  
}
boolean control_pump(int set_mode)
{
  if(inflate_check == true)
  {
    if(set_mode < 1)
    {
      analogWrite(E1,200);
      return false;
    }
    else inflate_check == false;
  }
  else 
  {
    analogWrite(E1,0);
    return true;
  }
}
int bluetooth_receive()
{
  if(BTSerial.available()>0)
  {
    return int(BTSerial.read());
  }
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
  if(constant_speed_check == true) analogWrite(E2,100);
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

    Serial.print("      Bluetooth's mode:");
    Serial.print(mode);
    
    last_print_time = millis();
    Serial.print("      Time:");
    Serial.println(last_print_time/1000);
  }
}

