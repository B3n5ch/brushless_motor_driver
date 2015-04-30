#include "Brushless_Motorshield.h"

static void startMotor();
static void setState();
static void readNextState();
static void brake();
static void writeOutputState(uint8_t phase, uint8_t state);
static void setDirection(int8_t direction);
static void setSpeedVal(uint8_t speed);
static void incSpeedVal(uint8_t val);
static void decSpeedVal(uint8_t val);
static void velocityControlTask();

/**
 * @brief internal speedValue for the PWM signal
 */
static volatile uint8_t _speed = 0;
/**
 * @brief the current velocity of the Motor in turns per second
 * @detail is updated avery AVG_LENGTH steps (6 steps are 1 turn)
 */
static volatile float _current_velocity = 0;
/**
 * @brief the current set velocity value (turns per second)
 */
static volatile float _velocity = 0;
/**
 * @brief used to measure the speed value by counting the timer2 overflows
 */
static volatile uint8_t _ovf_count = 0;

/**
 * @brief the count var for the speed measurement 
 */
static volatile uint8_t _avg_count = AVG_LENGTH;

/**
 * @brief count var for the startup procedure
 * @detail if _startup <> 0 then a startup procedure is needed
 */
static volatile uint8_t _startup = STARTUP_LENGTH;
/** 
 * @brief actual state var which tells the last known position of the motor axis
 */
static volatile uint8_t _state = 1;
/**
 * @brief turn direction 
 */
static volatile int8_t _direction = FORWARD;


/**
 * @brief Init Method to init Timer2, PCINT and Watchdog and the input output Ports
 */
void Brushless_Motorshield::init()
{
	//Set input Pins for Sensors IN/Without Pull Up
	DDR_SENS_U &= ~(1<<SENS_U);
	DDR_SENS_V &= ~(1<<SENS_V);
	DDR_SENS_W &= ~(1<<SENS_W);

	//Use external PULL Up's
	PORT_SENS_U &= ~(1<<SENS_U);
	PORT_SENS_V &= ~(1<<SENS_V);
	PORT_SENS_W &= ~(1<<SENS_W);
	
	//Set output Ports for Drivers
	DDR_U |= (1<<U);
	DDR_U_INH |= (1<<U_INH);
	DDR_V |= (1<<V);
	DDR_V_INH |= (1<<V_INH);
	DDR_W |= (1<<W);
	DDR_W_INH |= (1<<W_INH);


	PORT_U &= ~(1<<U);
	PORT_U_INH &= ~(1<<U_INH);
	PORT_V &= ~(1<<V);
	PORT_V_INH &= ~(1<<V_INH);
	PORT_W &= ~(1<<W);
	PORT_W_INH &= ~(1<<W_INH);

	cli();
	//Setup PWM for speed (torque) controll
	//TCCR2A mode
	TCCR2A |= (1<<WGM20)|(1<<WGM21);
	// clear TCNT2
	TCNT2 = 0;
	// enable timer 2 overflow and compare match a interrupt
	TIMSK2 |= (1<<TOIE2) | (1<<OCIE2A);// | (1<<OCIE2B);
	// set compare match register to zero
	OCR2A = 0;
	// prescaler = 1, 1 Timerstep = 500 ns, 255 Timersteps = 127.5 usm, fPWM = 7,8 kHz
	TCCR2B = (1<<CS21);

	//Initialize Pin Change interrupts
  	PCICR |= (1<<PCIE);
  	PCMSK1 |= (1<<SENS_U_PCINT)|(1<<SENS_V_PCINT)|(1<<SENS_W_PCINT);//0b00111000;
  
	//Enable WD with 500ms Timeout
	wdt_enable(WDTO_250MS);
	// init watchdog as timeout timer; 
	WDTCSR = (1<<WDCE) | (1<<WDIE);
	sei();

	//read State at startup
	readNextState(); 
}

///////////////////////////////////////////////////
/// private methods
///////////////////////////////////////////////////

/**
 * @brief reads the actual rotor position and sets the next outputstate to _state
 */
void readNextState()
{
	uint8_t val_u = (PIN_SENS_U>>SENS_U)&0x01;
	uint8_t val_v = (PIN_SENS_V>>SENS_V)&0x01;
	uint8_t val_w = (PIN_SENS_W>>SENS_W)&0x01;
	
	if(val_u == 1 && val_v == 0 && val_w == 1)
		_state = 1 + _direction;

	else if(val_u == 0 && val_v == 0 && val_w == 1)
		_state = 2 + _direction;

	else if(val_u == 0 && val_v == 1 && val_w == 1)
		_state = 3 + _direction;

	else if(val_u == 0 && val_v == 1 && val_w == 0)
		_state = 4 + _direction;

	else if(val_u == 1 && val_v == 1 && val_w == 0)
		_state = 5 + _direction;

	else if(val_u == 1 && val_v == 0 && val_w == 0)
		_state = 6 + _direction;
		
	if(_state == 7)
		_state = 1;
	else if(_state == 0)
		_state = 6;
	
}

/**
 * @brief writes the given driver State to the outputs
 */
void writeOutputState(uint8_t phase, uint8_t driverState)
{	
	if(phase == PHASE_U)
	{
		if(driverState == HIGH)
			PORT_U |= (1<<U);
		else
			PORT_U &= ~(1<<U);

		if(driverState != OFF)
			PORT_U_INH |= (1<<U_INH);
		else
			PORT_U_INH &= ~(1<<U_INH);
	}
	else if(phase == PHASE_V)
	{
		if(driverState == HIGH)
			PORT_V |= (1<<V);
		else
			PORT_V &= ~(1<<V);

		if(driverState != OFF)
			PORT_V_INH |= (1<<V_INH);
		else
			PORT_V_INH &= ~(1<<V_INH);
	}
	else if(phase == PHASE_W)
	{
		if(driverState == HIGH)
			PORT_W |= (1<<W);
		else
			PORT_W &= ~(1<<W);

		if(driverState != OFF)
			PORT_W_INH |= (1<<W_INH);
		else
			PORT_W_INH &= ~(1<<W_INH);
	}
}


/**
 * @brief increases the speed val by val
 */
void incSpeedVal(uint8_t val)
{
	if(_speed + val <= 255)
		_speed = _speed + val;
	OCR2A = _speed;
}

/**
 * @brief decreases the speed val by val
 */
void decSpeedVal(uint8_t val)
{
	if(_speed > val + MIN_SPEED_VAL)
		_speed = _speed - val;
	OCR2A = _speed;
}

/**
 * @brief sets the internal Speed val
 */
void setSpeedVal(uint8_t spd)
{
	if(spd == 0)
		_startup = STARTUP_LENGTH;
	_speed = spd;
	OCR2A = _speed;
}

/**
 * @brief controlles the _speed Value to turn the rotor with the given velocity
 */
void velocityControlTask()
{
	if (_current_velocity < _velocity - CONTROL_OFFSET_WIDE)
	{
		incSpeedVal(CONTROL_INC_FAST);
	}
	if (_current_velocity < _velocity - CONTROL_OFFSET_CLOSE)
	{
		incSpeedVal(CONTROL_INC_SLOW);
	}
	else if(_current_velocity > _velocity + CONTROL_OFFSET_WIDE)
	{
		decSpeedVal(CONTROL_INC_FAST);
	}
	else if(_current_velocity > _velocity + CONTROL_OFFSET_CLOSE)
	{
		decSpeedVal(CONTROL_INC_SLOW);
	}
}


/**
 * @brief Sets the turn Direction
 */
void setDirection(int8_t dir)
{
	if(dir > 0)
		_direction = FORWARD;
	else if(dir < 0)
		_direction = BACKWARD;
}


/**
 * @brief stopps the Motor
 */
void brake()
{
	writeOutputState(PHASE_U,OFF);
	writeOutputState(PHASE_V,OFF);
	writeOutputState(PHASE_W,OFF);
	_startup = STARTUP_LENGTH;
}


/**
 * @brief sets the State of privat var _state to the outputs
 */
void setState()
{
	cli();
	if(_state==1)
	{
		writeOutputState(PHASE_U,LOW);
		writeOutputState(PHASE_V,HIGH);
		writeOutputState(PHASE_W,OFF);
	}
	else if(_state==2)
	{
		writeOutputState(PHASE_U,OFF);
		writeOutputState(PHASE_V,HIGH);
		writeOutputState(PHASE_W,LOW);
	}
	else if(_state==3)
	{
		writeOutputState(PHASE_U,HIGH);
		writeOutputState(PHASE_V,OFF);
		writeOutputState(PHASE_W,LOW);
	}
	else if(_state==4)
	{	
		writeOutputState(PHASE_U,HIGH);
		writeOutputState(PHASE_V,LOW);
		writeOutputState(PHASE_W,OFF);
	}
	else if(_state==5)
	{	
		writeOutputState(PHASE_U,OFF);
		writeOutputState(PHASE_V,LOW);
		writeOutputState(PHASE_W,HIGH);
	}
	else if(_state==6)
	{	
		writeOutputState(PHASE_U,LOW);
		writeOutputState(PHASE_V,OFF);
		writeOutputState(PHASE_W,HIGH);
	}
	sei();
}


/**
 * @brief method to start the motor
 */
void startMotor()
{
	//Time delay between stateChange
	uint8_t state_delay = FORCED_MOVE_TIME_START;
	//Set Startup_Speed_Val
	if(_velocity != 0)
		_speed = STARTUP_SPEED_VAL;
	
	//Read actual state for Startup
	readNextState();
	
	//Startup Procedure
	while(_startup > 0)
	{
		_startup--;
			
		if(_startup % STARTUP_DIVIDER == 0)
			state_delay++;
			 
		setState();
		
		for(int i = 0; i < state_delay; i++)
		{
			_delay_ms(1);
		}
		
		_state += _direction;
		if(_state > 6)
			_state = 1;
		else if(_state < 1)
			_state = 6;
	}
}

///////////////////////////////////////////////////
/// public Methods
///////////////////////////////////////////////////
/**
 * @brief gets the current velocity in turns per second
 */
float Brushless_Motorshield::getVelocity()
{
	return _current_velocity * _direction;
}

/**
 * @brief sets the current velocity in turns per second
 */
void Brushless_Motorshield::setVelocity(float vel)
{
	if(vel < 0)
	{
		vel = vel * -1;
		_direction = BACKWARD;
	}
	else
		_direction = FORWARD;
		
	
	_velocity = vel;
	if(_velocity < MIN_SPEED)
		setSpeedVal(0);
	else if(_speed == 0)
		setSpeedVal((255.0 - MIN_SPEED_VAL) / (MAX_SPEED - MIN_SPEED) * vel);
	
	
	if(_startup > 0 && _speed > 0)
		startMotor(); 	
}

//////////////////////////////////////////////////////////////
/// Interrup Service Routines
//////////////////////////////////////////////////////////////
//If the rotor turns 60 degrees this interrupt appears
/**
 * @brief reads the next state and sets it; also the current speed val is computed and the _speed value is corrected
 */
ISR(PCINT_VECTOR) { 
	readNextState();

	if(_speed > 0)
		setState();
	else
		brake();

	_avg_count++;
	if(_avg_count==AVG_LENGTH)
	{
		_current_velocity = AVG_LENGTH / 6.0 * 7812.0 / (float)_ovf_count;
		_avg_count = 0;
		_ovf_count = 0;

		_startup=0;	 

		velocityControlTask(); 
		wdt_reset();
	
	}
}

/**
 * @brief PWM Task
 */
ISR(TIMER2_OVF_vect) {
	if(_speed > 0)
	{
		switch(_state) {
			case 1: 
				PORT_V |= (1<<V);
				PORT_U |= (1<<U_INH); 
			break; // U = PWM
			case 2: 
				PORT_V |= (1<<V);
				PORT_U |= (1<<W_INH);
			break; // V = PWM
			case 3: 
				PORT_V |= (1<<U);
				PORT_U |= (1<<W_INH);
			break; // V = PWM
			case 4: 
				PORT_V |= (1<<U);
				PORT_U |= (1<<V_INH); 
			break; // W = PWM
			case 5: 
				PORT_V |= (1<<W);
				PORT_U |= (1<<V_INH); 
			break; // W = PWM
			case 6: 
				PORT_V |= (1<<W);
				PORT_U |= (1<<U_INH); 
			break; // U = PWM
			default: 
			break;	
		}
	}
		
	if(_speed == 0 || _velocity == 0)
	{
		_speed = 0;
		_startup = STARTUP_LENGTH;
	}
	_ovf_count++;
}



/**
 * @brief PWM Task
 */
ISR(TIMER2_COMPA_vect) {
	switch(_state) {
		case 1: 
			PORT_V &= ~(1<<V);
			PORT_U &= ~(1<<U_INH); 
		break; // U = PWM
		case 2: 
			PORT_V &= ~(1<<V);
			PORT_U &= ~(1<<W_INH);
		break; // V = PWM
		case 3: 
			PORT_V &= ~(1<<U);
			PORT_U &= ~(1<<W_INH);
		break; // V = PWM
		case 4: 
			PORT_V &= ~(1<<U);
			PORT_U &= ~(1<<V_INH); 
		break; // W = PWM
		case 5: 
			PORT_V &= ~(1<<W);
			PORT_U &= ~(1<<V_INH); 
		break; // W = PWM
		case 6: 
			PORT_V &= ~(1<<W);
			PORT_U &= ~(1<<U_INH); 
		break; // U = PWM
		default: 
		break;	
		}
}

/** 
 * @brief watchdog isr: if this occurs the motor did either never start or was forcefully stopped
 */
ISR(WDT_vect) {
	wdt_reset();
	WDTCSR |= (1<<WDCE) | (1<<WDIE); // reenable interrupt to prevent system reset
	_startup = STARTUP_LENGTH;	
	sei();

	_avg_count = 0;
	_current_velocity = 0;

	if(_startup > 0 && _speed > 0)
		startMotor(); 	
}
