#ifndef BRUSHLESS_H_
#define BRUSHLESS_H_

#define HIGH  1
#define LOW   2
#define OFF   3

//Output 
//LX Motorshield Arduino
#define PHASE_U        1
#define PHASE_U_INH    2
#define PHASE_V        3
#define PHASE_V_INH    4
#define PHASE_W        5
#define PHASE_W_INH    6

#define DDR_U		DDRD
#define DDR_U_INH	DDRD
#define DDR_V		DDRD
#define DDR_V_INH	DDRD
#define DDR_W		DDRB
#define DDR_W_INH	DDRD

#define PORT_U		PORTD
#define PORT_U_INH	PORTD
#define PORT_V		PORTD
#define PORT_V_INH	PORTD
#define PORT_W		PORTB
#define PORT_W_INH	PORTD

#define U		PD2
#define U_INH		PD3
#define V		PD4
#define V_INH		PD5
#define W		PB0
#define W_INH		PD7
//Inputs
#define DDR_SENS_U	DDRC
#define DDR_SENS_V	DDRC
#define DDR_SENS_W	DDRC

#define PORT_SENS_U	PORTC
#define PORT_SENS_V	PORTC
#define PORT_SENS_W	PORTC

#define PIN_SENS_U	PINC
#define PIN_SENS_V	PINC
#define PIN_SENS_W	PINC

#define SENS_U		PC3
#define SENS_V		PC4
#define SENS_W		PC5

#define PCMSK_REG	PCMSK1
#define SENS_U_PCINT	PCINT11
#define SENS_V_PCINT	PCINT12
#define SENS_W_PCINT	PCINT13
#define PCINT_VECTOR	PCINT1_vect
#define PCIE		PCIE1

//Direction
#define FORWARD      1
#define BACKWARD    -1

//Speed Estimation
#define MAX_SPEED               500.0
#define MIN_SPEED               70.0
#define MIN_SPEED_VAL           5.0
#define AVG_LENGTH              10

//Speed Controll
#define CONTROL_OFFSET_CLOSE	5.0
#define CONTROL_OFFSET_WIDE	50.0
#define CONTROL_INC_SLOW	1
#define CONTROL_INC_FAST	10

//Startup
#define STARTUP_LENGTH          50
#define STARTUP_DIVIDER         10
#define FORCED_MOVE_TIME_START  0
#define STARTUP_SPEED_VAL	120

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>

class Brushless_Motorshield {
public:
	static float getVelocity();
	static void setVelocity(float vel);
	static void init();
};
#endif
