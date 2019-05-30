/**************************************************************************************/
/***************                        Type                       ********************/
/**************************************************************************************/

typedef struct {
	//uint8_t currentSet;
	int16_t accZero[3];
	//int16_t magZero[3];
	//uint16_t flashsum;
	//uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE !
} global_conf_t;

typedef struct
{
	//int16_t  accSmooth[3];
	int16_t  gyroData[3];
	//int16_t  magADC[3];
	int16_t  gyroADC[3];
	int16_t  accADC[3];
} imu_t;

enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  /*AUX1,
  AUX2,
  AUX3,
  AUX4,
  AUX5,
  AUX6,
  AUX7,
  AUX8*/
};
