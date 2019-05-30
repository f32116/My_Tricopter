#include "Arduino.h"
#include "Config.h"
#include "Output.h"
#include "Tricopter.h"

int value = 1500;

void setup()
{
  Serial.begin(9600);
  Serial.println("0");
  Output_init();
  Serial.println("1");
  //Serial.println(atomicServo);
}
char cmd[9] ;
void loop()
{
  if(Serial.available() > 0 )
  {
    Serial.readBytes(cmd,9);
    value = atoi(cmd);
    Serial.println(value);
    servo = value;
	Serial.println((servo - 1000) >> 2);
  }
  Servos_write();
  //Serial.println(atomicServo);

}
