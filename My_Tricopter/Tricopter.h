/**************************************************************************************/
/***************                       Output                      ********************/
/**************************************************************************************/
#include "stdint.h"
#include "Type.h"


extern uint16_t calibratingA;
extern uint16_t calibratingG;

extern imu_t imu;
extern global_conf_t global_conf;
extern int16_t gyroZero[3];

extern int16_t motor[3];
extern int16_t servo;

extern int16_t  i2c_errors_count;
