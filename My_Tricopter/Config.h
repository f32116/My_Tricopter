/**************************************************************************************/
/***************                       Config                      ********************/
/**************************************************************************************/

/*****************************    Motor minthrottle    ********************************/
/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
   This is the minimum value that allow motors to run at a idle speed  */
#define MINTHROTTLE 1150

/*****************************    Motor maxthrottle    ********************************/
/* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
#define MAXTHROTTLE 2000

/******************************    Mincommand          ********************************/
/* this is the value for the ESCs when they are not armed in some cases, 
   this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
#define MINCOMMAND  1000



#define YAW_DIRECTION 1
