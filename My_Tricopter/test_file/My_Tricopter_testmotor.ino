#include "Arduino.h"
#include "Config.h"
#include "Output.h"
#include "Tricopter.h"

int value = 1150;

void setup()
{
  
  Serial.begin(9600);
  Serial.println("0");
  Output_init();
  
  motor[0] = value;
  motor[1] = value;
  motor[2] = value;
  Serial.println("1");
  Motors_write();
  /*motor[0] = 1850;
  motor[1] = 1850;
  motor[2] = 1850;*/
  
  /*for(int i = 0; i < 10000; i++)
  {
    for(int j=0;j<20;j++)
    {
      Motors_write();
    }
  }
  motor[0] = 1150;
  motor[1] = 1150;
  motor[2] = 1150;
  Serial.println("2");
  for(int i = 0; i < 10000; i++)
  {
    for(int j=0;j<10;j++)
    {
      Motors_write();
    }
  }
  Serial.println("3");*/
  delay(5000);
  Serial.println("2");
}
char cmd[9] ;
void loop()
{
  //Serial.println("3");
	//Servos_write();
  if(Serial.available() > 0 )
  {
    //value = Serial.parseInt();
    Serial.readBytes(cmd,9);
    value = atoi(cmd);
    Serial.println(value);
    //Serial.println(value);
    motor[0] = value;
    motor[1] = value;
    motor[2] = value;
  }
  Motors_write();
  

}
