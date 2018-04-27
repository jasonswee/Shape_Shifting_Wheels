/*
  300007_Shape_Shifting_Wheels
  Created by Jason Swee, April 05, 2018.
*/

#include "Arduino.h"
#include <Encoder.h> //Encoder library
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

//Set the Bluetooth pins below
SoftwareSerial BTSerial(10, 11); //RX,TX
#define BLUETOOTH_SPEED 9600

//LED Strip Settinga
#define LED_pin 3
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, LED_pin);

//Set the encoder pins below
#define ENCODER_PINA 2
#define ENCODER_PINB 4
#define POS_TO_VEL_CONST 1

//Set the Rain threshold
#define RAIN_PIN A3
#define RAIN_THRESHOLD 850

#define RESIST_PIN A5
#define RESIST_RANGE 2 //Indicates the + - buffer error between resistance range
#define MODE_0 245 //Indicates the resistance value to trigger mode 1
#define MODE_1 238 //Indicates the resistance value to trigger mode 2

//Set pushbutton pin
#define PUSH_PIN 7

//Encoder myEnc(ENCODER_PINA, ENCODER_PINB); //Set pins for encoder

//State
int state = 0; 

//Simulated speed function
int E2 = 6; //M2 speed control
int M2 = 7; //M2 Directional control

//Pump function
int E1 = 5;//M1 speed control
int M1 = 4;//M1 Directional control

//Mode to for receiving BT
int mode = 0;

//Push button
boolean inflate_check = false;
boolean button_status = LOW;

//Rain module 
int rain_value = 0;

//Needed for printing time
unsigned long last_print_time = 0;

//Check resistor
int inflate_mode = 0;

//Romeo's LED
int ledpin = 13;

//***Obsolete variables***
//int count = 0;
//int desired_spd = 0;

//Encoder reading function
//unsigned long old_time = 0;
//unsigned long new_time = 0;
//unsigned long velocity = 0;
//unsigned long encoded_position = 0;
//unsigned long last_poll_time = 0;
//int debounce_delay = 10;
//***Obsolete variables***

//The setup function is called once at startup of the sketch
void setup()
{
  //pinMode(ledPin, OUTPUT); 
  
  // Add your initialization code here
  Serial.begin(9600);
  BTSerial.begin(9600);
  //Needed for print
  last_print_time = millis();
  
  //Initialise motor
  pinMode(E2, OUTPUT); 
  pinMode(M2, OUTPUT); 
  digitalWrite(M2,LOW);

  //Rain
  pinMode(RAIN_PIN,INPUT);

  //Push
  pinMode(PUSH_PIN,INPUT);

  //Initialise Pump
  pinMode(E1, OUTPUT); 
  pinMode(M1, OUTPUT); 
  digitalWrite(M1,LOW);

  //LED
  pinMode(LED_pin,OUTPUT);
  pinMode(ledpin,OUTPUT);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off

  //Obsolete
  //last_poll_time = millis();
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here
  
  Is_inflate_checked(); //checks user inputs and changes state if have input
  Is_mode_inflated(); //reads flex sensor reading and checks if fully inflated
  if (state == 0)
  {
    run_motor(170); //Input speed of motor
    run_pump(0); //Input speed of pump
  }
  //Simulate changing speed
  else if(state == 1)
  {
    run_motor(170); //Input speed of motor
    run_pump(150);
  }

  //LED
  LED_code();
  
  //Capture received mode
  mode = bluetooth_receive();
  
  //Print every 1 second, argument milliseconds of frequency
  print_status(500); 
  
  //***Obsolete code ***
  //Print every 0.1 second, argument milliseconds of frequency
  //Read motor, obsolete code
  //poll_motor_spd(100);
  
  //control_pump(mode);
  //***Obsolete code ***
}
void Is_inflate_checked()
{
  read_user_command();
  if (inflate_check == true) state = 1;
  else state = 0;
}
void Is_mode_inflated()
{
  if (mode == 1)//Inflated 
  {
    inflate_check = false;
    state = 0; 
  }
//***Obsolete code***
//  int resistance = analogRead(RESIST_PIN);
//  Serial.print("Resistance Value: ");
//  Serial.print(resistance);
//  Serial.println(" ");
//  delay(500);
//  if (resistance >=(MODE_0-RESIST_RANGE) && resistance <= (MODE_0+RESIST_RANGE) || resistance > MODE_0) mode = 0;
//  else if (resistance >=(MODE_1-RESIST_RANGE)&& resistance <=(MODE_1+RESIST_RANGE)) mode = 1;
//  Serial.print("Mode: ");
//  Serial.println(mode);
//***Obsolete code***
}
void read_user_command()
{
  rain_value = analogRead(RAIN_PIN);

  button_status = digitalRead(PUSH_PIN);
  if(rain_value < RAIN_THRESHOLD){
    inflate_check = true;
  }
  if(button_status == HIGH){
    inflate_check = true;
  }
  delay(100); //Software debounce for push button
}
int bluetooth_receive()
{
  if(BTSerial.available()>0)
  {
    int error_check = int(BTSerial.read());
    if (error_check>1)return int(mode);
    else return error_check;
  }
}

void run_motor(int motor_speed)
{
  analogWrite(E2,motor_speed);
//***Obsolete code ***
//    analogWrite(E2,desired_spd);//desired_spd);
//    count++;
//    if(count>=2000)desired_spd+=10;
//    if(desired_spd>=200)desired_spd=0;
//***Obsolete code ***
}
void run_pump(int pump_speed)
{
  analogWrite(E1,pump_speed);
}
void print_status(int print_every_x_millis)
{
  if(millis()- last_print_time >= print_every_x_millis)
  {
    
    Serial.print("Rain value: ");
    Serial.println(rain_value);
//
//    Serial.print("Velocity:");
//    Serial.print(velocity);
//    
//    Serial.print("      Encoder:");
//    Serial.print(encoded_position);
//
//    Serial.print("      Bluetooth's mode:");
//    Serial.print(mode);
//
    
    Serial.print("      Push button status:");
    Serial.print(button_status);

    Serial.print("      Mode:");
    Serial.print(mode);

    Serial.print("      State:");
    Serial.print(state);
    
    last_print_time = millis();
    Serial.print("      Time:");
    Serial.println(last_print_time/1000);
  }
}

void LED_code(){
  
//  if (mode == 0 && state == 1){ //tyre not fully inflated and pump activated
//    lighting(0,255,0,25);
//  }
//  else if (mode == 1 && state == 0){ //tyre fully inflated and pump not activated
//    lighting(0,0,255,25);
//  }
//  else{
//    lighting(255,0,0,25);
  if (state == 0){
    if (mode > 0){;
      lighting(0,0,255,25); // blue
    }
    else{
      lighting(255,0,0,25); //red
    }
  }
  else{
    if (inflate_check == true){
      lighting(0,255,0,25); // green
    }
  }
}

void lighting(int red, int blue,int green,int brightness){
  strip.setPixelColor(0,red,blue,green);
  strip.setPixelColor(1,red,blue,green);
  strip.setPixelColor(2,red,blue,green);
  strip.setPixelColor(3,red,blue,green);
  strip.setPixelColor(4,red,blue,green);
  strip.setPixelColor(5,red,blue,green);
  strip.setPixelColor(6,red,blue,green);
  strip.setPixelColor(7,red,blue,green);
  strip.setBrightness(brightness);
  strip.show();
}

//***Obsolete code ***
//void control_pump(int set_mode)
//{
//  if(inflate_check == true)
//  {
//    if(set_mode < 1)  analogWrite(E1,200);//Check if it is deflated
//    else inflate_check = false;
//  }
//  else analogWrite(E1,0);
//}
//void poll_motor_spd(int poll_every_x_millis)
//{
//  if(millis()-last_poll_time >= poll_every_x_millis)
//  {
//    encoded_position = myEnc.read();
//    velocity = encoded_position*POS_TO_VEL_CONST/(poll_every_x_millis);
//    last_poll_time = millis();
//    myEnc.write(0);
//    delay(debounce_delay);
//  }
//}
//***Obsolete code ***
