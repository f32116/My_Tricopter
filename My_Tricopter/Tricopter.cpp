/**************************************************************************************/
/***************                      Tricopter                    ********************/
/**************************************************************************************/
#include "Arduino.h"
#include "config.h"
#include "Output.h"
#include "stdint.h"
#include "sensors.h"
#include "Type.h"

/*uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;*/
uint16_t calibratingA = 0;
uint16_t calibratingG;

// **************
// gyro+acc IMU
// **************
int16_t gyroZero[3] = { 0,0,0 };
imu_t imu;

// ************************
// EEPROM Layout definition
// ************************
global_conf_t global_conf;

// *************************
// motor and servo functions
// *************************
//int16_t axisPID[3];
int16_t motor[3];
int16_t servo = { 1450 };

int16_t  i2c_errors_count = 0;

/*void setup()
{
	Output_init();
	//readGlobalSet();

	Sensors_init();
	previousTime = micros();
}*/
