#include "Arduino.h"
#include "Config.h"
#include "Output.h"
#include "Tricopter.h"
#include "Sensors.h"

void setup()
{
  Serial.begin(9600);
  Serial.println("0");
  Output_init();
  Sensor_init();
  calibratingG = 512;
  Serial.println("1");
  //Serial.println(atomicServo);
}

void loop()
{
  /*ACC_getADC();
  for(int i=0;i<3;i++)
  {
    Serial.print(imu.accADC[i]);
    Serial.print(" ");
  }
  Serial.println(" ");*/
  Gyro_getADC();
  for(int i=0;i<3;i++)
  {
    Serial.print(imu.gyroADC[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}
