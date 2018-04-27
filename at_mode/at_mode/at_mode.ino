#include <SoftwareSerial.h>
SoftwareSerial EEBlue(10, 11); // RX | TX
 
void setup()
{
  //uncomment below to initiate AT mode
  //pinMode(9, OUTPUT);
  //digitalWrite(9, HIGH);
  Serial.begin(9600);
  EEBlue.begin(38400);  //Baud Rate for command Mode. 
  Serial.println("Enter AT commands!");
}
 
void loop()
{
  // Feed any data from bluetooth to Terminal.
  if (EEBlue.available())
    Serial.write(EEBlue.read());
 
  // Feed all data from termial to bluetooth
  if (Serial.available())
    EEBlue.write(Serial.read());
}
