/**************************************************************************************/
/***************                       Output                      ********************/
/**************************************************************************************/
#include "Tricopter.h"
#include "Config.h"
#include "Arduino.h"
#include "Output.h"

volatile uint8_t atomicServo = { 125 };

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void Motors_write()
{
	OCR1A = motor[0] >> 3; //  pin 9
	OCR1B = motor[1] >> 3; //  pin 10
	OCR2A = motor[2] >> 3; //  pin 11
}

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void Servos_write()
{
	atomicServo = (servo - 1000) >> 2;
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<3;i++) {
    motor[i]=mc;
  }
  Motors_write();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void Output_init()
{
	/****************            mark all PWM pins as Output             ******************/
	uint8_t PWM_PIN[3] = { 9,10,11 };
	for (uint8_t i = 0; i < 3; i++)
	{
		pinMode(PWM_PIN[i], OUTPUT);
	}

	/**************    Specific PWM Timers & Registers for the atmega328P    **************/
	TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
	TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
	TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A

	writeAllMotors(MINCOMMAND);
	delay(300);

	/**************               Initialize the PWM Servos               *****************/
	pinMode(3, OUTPUT);
	TCCR0A = 0; // normal counting mode
	TIMSK0 |= (1 << OCIE0A); // Enable CTC interrupt
}
/**************************************************************************************/
/************              Servo software PWM generation             ******************/
/**************************************************************************************/

// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us

ISR(TIMER0_COMPA_vect)
{
	static uint8_t state = 0; // indicates the current state of the chain
	if (state == 0)
	{
		PORTD |= 1 << 3;
		OCR0A += 250; // wait 1000us
		state++; // count up the state
	}
	else if (state == 1)
	{
		OCR0A += atomicServo; // load the servo's value (0-1000us)
		state++; // count up the state
	}
	else 
	{
		PORTD &= ~(1 << 3);
		OCR0A += 250;
		if (state < 30)
		{
			state += 2;
		}
		else
		{
			state = 0;
		}
	}
}


/*void mixTable()
{
	//#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z
    //#define SERVODIR(n,b) ((conf.servoConf[n].rate & b) ? -1 : 1)

	motor[0] = PIDMIX(0, +4 / 3, 0); //REAR
	motor[1] = PIDMIX(-1, -2 / 3, 0); //RIGHT
	motor[2] = PIDMIX(+1, -2 / 3, 0); //LEFT
	servo = (SERVODIR(5, 1) * axisPID[YAW]) + get_middle(5); //REAR

	/****************                normalize the Motors values                ******************/      //?
/*	maxMotor = motor[0];
	for (i = 1; i < 3; i++)
	{
		if (motor[i] > maxMotor)
		{
			maxMotor = motor[i];
		}
	}		
	for (i = 0; i < 3; i++) 
	{
		if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
		{
			motor[i] -= maxMotor - MAXTHROTTLE;
		}
		motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
	}
}*/
