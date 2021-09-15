/*
 DACNCRouter.cpp - CNC Router library for Arduino v. 0.1
  
 Original library     (0.1) by Alexander 'Derzhiarbuz' Gubanov.
 
 Drives two- or three-dimension CNC router using G-Code commands
 
 Each motor drived in half-step mode by default. Motors controlled by 2 pins - 'step' and 'direction' ('cw'), you can use L297 circuit to implement this type of control.
 
 Each axis can have from zero to two limit sensors (left and right). If you have no edge sensor, you should set negative value for it's pin.
 
 G_Code commands implemented:
 
 G01 - Linear interpolation (example: G01 X0. Y0. Z100 F200  or  15G01X10.5Z-0.034)
 M30 - End of program
 */


#include "Arduino.h"
#include "DACNCRouter.h"

/*
 * constructor.
 * Sets router status to Uninitialized. For initialization instrument and at least X and Y axis should be defined.
 */
DACNCRouter::DACNCRouter()
{
    _xAxis.enabled = 0;
    _yAxis.enabled = 0;
    _zAxis.enabled = 0;
    
    _instrument.enabled = 0;
    

    _status = CNCUninitialized;
    speed = .0;
	_epsilon = .0000001;
    _prepaired = 0;
}

/*
 Sets the X axis.
 Use negative pin value for left_limit_pin or right_limit_pin if you have no apropriative sensor
 
 */
void DACNCRouter::setAxisX(int motor_steps_number, int motor_step_pin, int motor_cw_pin, int motor_enable_pin, double mm_per_step, double axis_length_mm, int left_limit_pin, int right_limit_pin)
{
    if(_status == CNCProcessing) return;
    
    _setAxis(&_xAxis, motor_steps_number, motor_step_pin, motor_cw_pin, motor_enable_pin, mm_per_step, axis_length_mm, left_limit_pin, right_limit_pin);
    
    if(_xAxis.enabled && _yAxis.enabled)
        _status = CNCIdle;
}

/*
 Sets the Y axis
 Use negative pin value for left_limit_pin or right_limit_pin if you have no apropriative sensor
 
 */
void DACNCRouter::setAxisY(int motor_steps_number, int motor_step_pin, int motor_cw_pin, int motor_enable_pin, double mm_per_step, double axis_length_mm, int left_limit_pin, int right_limit_pin)
{
    if(_status == CNCProcessing) return;
    
    _setAxis(&_yAxis, motor_steps_number, motor_step_pin, motor_cw_pin, motor_enable_pin, mm_per_step, axis_length_mm, left_limit_pin, right_limit_pin);
    
    if(_xAxis.enabled && _yAxis.enabled)
        _status = CNCIdle;
}

/*
 Sets the Z axis
 Use negative pin value for left_limit_pin or right_limit_pin if you have no apropriative sensor
 
 */
void DACNCRouter::setAxisZ(int motor_steps_number, int motor_step_pin, int motor_cw_pin, int motor_enable_pin, double mm_per_step, double axis_length_mm, int left_limit_pin, int right_limit_pin)
{
    if(_status == CNCProcessing) return;
    
    _setAxis(&_zAxis, motor_steps_number, motor_step_pin, motor_cw_pin, motor_enable_pin, mm_per_step, axis_length_mm, left_limit_pin, right_limit_pin);
}


/*
 Sets instrument
 
 If instrument switched on by hands instrument_on_pin should be negative
 
 */
void DACNCRouter::setWorkingInstrument(int instrument_on_pin, double instrument_radius)
{
    if(_status == CNCProcessing) return;
    
    if(instrument_radius<0) instrument_radius = 0;
    _instrument.enabled = 1;
    _instrument.radius = instrument_radius;
    _instrument.on_pin = instrument_on_pin;
}

/*
 calls every program runloop step
 
 */
static long xSteps = 0;
static long xAcc = 0;
static long xDecc = 0;
static long sLeft = 0;

void DACNCRouter::act()
{
 //   static int iii = 0;
    static double dMls;
	if(_status != CNCProcessing) return;

	/*static double maxTime = .0;
    double currDMicros;
	unsigned long micros_1;
	unsigned long micros_2;

    micros_1 = micros();  */
 /*   iii++;
    if(iii==5000)
    {
        if(_xAxis.steps_left!=0 || _yAxis.steps_left!=0 || _zAxis.steps_left!=0)
        {
            iii=0;
			Serial.print(" X ");
			Serial.println(_xAxis.steps_left);
			Serial.print(" ");
            Serial.print(" Y ");
            Serial.println(_yAxis.steps_left);
            Serial.print(" ");
            Serial.print(" Z ");
            Serial.println(_zAxis.steps_left);
            Serial.print(" ");
        }
    }
  */  
    if(_xAxis.steps_left==0 && _yAxis.steps_left==0 && _zAxis.steps_left==0)
	{
		//if(_status != CNCIdle)
		/*	Serial.print("ES ");
            Serial.print(xAcc);
			Serial.print("  ");
			Serial.print(xDecc);
			Serial.print("  ");
			Serial.print(sLeft - xDecc);
            Serial.print("  ");
			Serial.println(_speed_multiplier);  */
		_status = CNCIdle;
        //digitalWrite(_xAxis.enable_pin, LOW);
        //digitalWrite(_yAxis.enable_pin, LOW);
        //digitalWrite(_zAxis.enable_pin, LOW);
    }
    else
    {
	  /*	unsigned long mls = micros();
		double dMls;
        
        if(mls >= _last_millis) dMls = mls - _last_millis;
        else
        {
            dMls = mls + (0xFFFFFFFF - _last_millis);
        }
		_last_millis = mls;  */
		dMls = 50.;
        if(_accelerated) { //acceleration/decceleration
			dMls *= _speed_multiplier;
            /*if(_speed_multiplier < 1.0) {
                _speed_multiplier+=_speed_increment;
                if(_speed_multiplier > 1.0)
                    _speed_multiplier = 1.0;
            }*/
	   	}

		_axisAct(&_xAxis, dMls);
		_axisAct(&_yAxis, dMls);
		_axisAct(&_zAxis, dMls);
	}

	/*micros_2 = micros();

	 if(micros_2 >= micros_1) currDMicros = micros_2 - micros_1;
		else
		{
			currDMicros = micros_2 + (0xFFFFFFFF - micros_1);
		}

	if(currDMicros > maxTime) {
		maxTime = currDMicros;
		Serial.print("EMcs ");
		Serial.println(currDMicros);
	}  */
}


/*
 Gets router status (see DACNCStatus enum)
 
 */
DACNCStatus DACNCRouter::status()
{
    return _status;
}

/*
 Gets code version
 
 */
int DACNCRouter::version()
{
    return 2;
}

int DACNCRouter::prepaired()
{
    return _prepaired;
}

void DACNCRouter::lineTo(double x, double y, double z, int next, double x1, double y1, double z1, double f1)
{
	_lineTo(x,y,z,next,x1,y1,z1,f1);
}

void DACNCRouter::setCurrentPosition(double x, double y, double z)
{
	_xAxis.position = x;
	_xAxisPrep.position = x;
	_yAxis.position = y;
	_yAxisPrep.position = y;
	if(_zAxis.enabled)
	{
		_zAxis.position = z;
		_zAxisPrep.position = z;
	}
}

void DACNCRouter::setInstrumentPower(int p)
{
    if(_instrument.enabled)
        analogWrite(_instrument.on_pin, p);
}

void DACNCRouter::setSpeed(double s) {
    speed = s;
    if(speed > _max_start_speed) {
        _accelerated = 1;
		_speed_increment = _one_step_accel/speed;
		//Serial.print("ESspeed ");
		//Serial.println(_fastest_axis->steps_left);
	   /*	Serial.print("OSA: ");
		Serial.println(_one_step_accel);
		Serial.print("Speed: ");
		Serial.println(speed);
        Serial.print("Speed increment: ");
        Serial.println(_speed_increment*10000);  */
    }
    else {
        _accelerated = 0;
    }
}

void DACNCRouter::setMaxStartSpeed(double s) {
    _max_start_speed = s;
    setSpeed(speed);
}

//PRIVATE

/*
 Sets axis
 Use negative pin value for left_limit_pin or right_limit_pin if you have no apropriative sensor
 
 */
void DACNCRouter::_setAxis(DACNCAxis *axis, int motor_steps_number, int motor_step_pin, int motor_cw_pin, int motor_enable_pin, double mm_per_step, double axis_length_mm, int left_limit_pin, int right_limit_pin)
{
    if(motor_step_pin<0 || motor_cw_pin<0 || axis_length_mm<=_epsilon || motor_steps_number<=0 || abs(mm_per_step)<_epsilon) axis->enabled = 0;
    
    axis->enabled = 1;
    axis->steps = motor_steps_number;
    axis->step_pin = motor_step_pin;
    axis->cw_pin = motor_cw_pin;
    axis->enable_pin = motor_enable_pin;
    axis->end_pin1 = 0;
    axis->left_limit_pin = left_limit_pin;
    axis->right_limit_pin = right_limit_pin;
    axis->steps_per_mm = 1.0/mm_per_step;
    axis->length = axis_length_mm;
    axis->position = .0;
    
    axis->steps_left = 0;
    axis->pin_level = 0;
}


void DACNCRouter::prepareLineTo(double x, double y, double z, double f, int next, double x1, double y1, double z1, double f1)
{
	if(_prepaired) return;

	if(f <= _epsilon)
	{
        return;
    }
    
    double dx, dy, dz, length;
    double time_needed;
    
	dx = x-_xAxisPrep.position;
	_xAxisPrep.is_left = dx<0?1:0;
	dy = y-_yAxisPrep.position;
    _yAxisPrep.is_left = dy<0?1:0;
    
    if(_zAxis.enabled)
    {
		dz = z-_zAxisPrep.position;
        _zAxisPrep.is_left = dz<0?1:0;
    }
    else
        dz = .0;
    
    length = sqrt(dx*dx + dy*dy + dz*dz);
    if(length<=_epsilon)
    {
        return;
    }
    
    time_needed = length / f;
    
	_xAxisPrep.steps_left = abs(dx)*_xAxis.steps_per_mm;
	if(_xAxisPrep.is_left)
	{
		_xAxisPrep.position -= _xAxisPrep.steps_left/_xAxis.steps_per_mm;
		xSteps += _xAxisPrep.steps_left;
	}
	else
	{
		_xAxisPrep.position += _xAxisPrep.steps_left/_xAxis.steps_per_mm;
		xSteps -= _xAxisPrep.steps_left;
	}
	_fastest_axisPrep = &_xAxisPrep;
	_xAxisPrep.millis_per_step = time_needed*1000000.0/_xAxisPrep.steps_left;
	_xAxisPrep.last_step = 0;

	_yAxisPrep.steps_left = abs(dy)*_yAxis.steps_per_mm;
	if(_yAxisPrep.is_left)
		_yAxisPrep.position -= _yAxisPrep.steps_left/_yAxis.steps_per_mm;
	else
		_yAxisPrep.position += _yAxisPrep.steps_left/_yAxis.steps_per_mm;
	if(_yAxisPrep.steps_left > _fastest_axisPrep->steps_left)
	 _fastest_axisPrep = &_yAxisPrep;
	_yAxisPrep.millis_per_step = time_needed*1000000.0/_yAxisPrep.steps_left;
	_yAxisPrep.last_step = 0;
        
	if(_zAxis.enabled)
    {
		_zAxisPrep.steps_left = abs(dz)*_yAxis.steps_per_mm;
		if(_zAxisPrep.is_left)
			_zAxisPrep.position -= _zAxisPrep.steps_left/_zAxis.steps_per_mm;
		else
			_zAxisPrep.position += _zAxisPrep.steps_left/_zAxis.steps_per_mm;
		if(_zAxisPrep.steps_left > _fastest_axisPrep->steps_left)
		_fastest_axisPrep = &_zAxisPrep;
		_zAxisPrep.millis_per_step = time_needed*1000000.0/_zAxisPrep.steps_left;
		_zAxisPrep.last_step = 0;
    }
    else
        _zAxisPrep.steps_left = 0;
    
    //acceleration
    static double last_cosa = .0, last_speed = .0;
	static double start_speed, finish_speed;
	long tmp;
    double cosa = .0;
    if(next) { //next point given
        double dx1, dy1, dz1, length1;
        dx1 = x1-x;
        dy1 = y1-y;
        if(_zAxis.enabled)
            dz1 = z1-z;
        else
            dz1 = .0;
        length1 = sqrt(dx1*dx1+dy1*dy1+dz1*dz1);
		if(length*length1 > 0.000001)
			cosa = (dx*dx1+dy*dy1+dz*dz1)/(length*length1);
        if(cosa<.0) cosa=.0;

		start_speed = (last_speed-_max_start_speed) *last_cosa + _max_start_speed; //start speed component we already have
		if(start_speed < _max_start_speed) start_speed = _max_start_speed;
		finish_speed = (speed*cosa + 1.1*_max_start_speed)/1.1; //finish speed
		//finish_speed = _max_start_speed;
		last_cosa  = cosa;
    }
	else {
		start_speed = (last_speed-_max_start_speed) *last_cosa + _max_start_speed;
        //start_speed = _max_start_speed;
        finish_speed = _max_start_speed;
    }
	last_speed = finish_speed; //it shall be less if it won't be accelerated enough (see bellow)
    if(_accelerated) {
        
		static int accSteps = (f-start_speed)/_one_step_accel; //number of steps of acceleration
        static int underaccSteps = 0; //number of steps whish should be accelerated but they weren't
		if(accSteps<0) accSteps = 0;

		_speed_multiplierPrep = start_speed/f;
		if(_speed_multiplierPrep>1.) _speed_multiplierPrep = 1.;
        
		_decceleration_stepsPrep = (f-finish_speed)/_one_step_accel;
        //calculating true decceleration steps
		tmp = accSteps+_decceleration_stepsPrep-_fastest_axisPrep->steps_left;
        if(tmp>0) { //if decceleration starts before max speed reached
			_decceleration_stepsPrep = (2*_decceleration_stepsPrep-tmp)/2;
        }
		if(_decceleration_stepsPrep<=0) { //if no deccelleration check if we do not reach finish_speed
			_decceleration_stepsPrep=0;
			if(accSteps > _fastest_axisPrep->steps_left) {
				underaccSteps = accSteps - _fastest_axisPrep->steps_left;
				last_speed = start_speed + (accSteps-underaccSteps)*_one_step_accel;
				if(last_speed > finish_speed) last_speed = finish_speed;
			}
		}

		if(_fastest_axisPrep == &_xAxisPrep)
			_fastest_axisPrep = &_xAxis;
		else if(_fastest_axisPrep == &_yAxisPrep)
			_fastest_axisPrep = &_yAxis;
		else if(_fastest_axisPrep == &_zAxisPrep)
			_fastest_axisPrep = &_zAxis;

		xAcc = accSteps-underaccSteps;
		xDecc = _decceleration_steps;
		sLeft = _fastest_axisPrep->steps_left;
		//_speed_multiplier = _max_start_speed/speed;
		//_decceleration_steps = (speed-_max_start_speed)/_one_step_accel;
		//if(_decceleration_steps*2 > _fastest_axis->steps_left)
		//    _decceleration_steps = _fastest_axis->steps_left/2-1;
	}

    _prepaired = 1;

	//Serial.print("E");
	//Serial.print(_xAxisPrep.steps_left);
	//Serial.println(_xAxis.millis_per_step);
	//Serial.println(_xAxis.steps_left);
	//Serial.print("EY");
	//Serial.print(_yAxis.position);
	//Serial.print(" ");
	//Serial.print(y);
	//Serial.print(" ");
	//Serial.println(_yAxis.steps_left);
	//Serial.print("EZ");
	//Serial.print(_zAxis.position);
	//Serial.print(" ");
	//Serial.print(z);
	//Serial.print(" ");
	//Serial.println(_zAxis.steps_left);

}

void DACNCRouter::_lineTo(double x, double y, double z, int next, double x1, double y1, double z1, double f1)
{
	//if route parameters not prepaired yet - prepare
	if(!_prepaired)
	{
		prepareLineTo(x,y,z,speed,next,x1,y1,z1,f1);
        Serial.println("EPrep");
	}

    //copying data from prepaired route
	_xAxis.is_left = _xAxisPrep.is_left;
	_xAxis.steps_left = _xAxisPrep.steps_left;
	_xAxis.millis_per_step = _xAxisPrep.millis_per_step;
	_xAxis.position = _xAxisPrep.position;
	_xAxis.last_step = _xAxisPrep.last_step;

	_yAxis.is_left = _yAxisPrep.is_left;
	_yAxis.steps_left = _yAxisPrep.steps_left;
	_yAxis.millis_per_step = _yAxisPrep.millis_per_step;
	_yAxis.position = _yAxisPrep.position;
	_yAxis.last_step = _yAxisPrep.last_step;

	_zAxis.is_left = _zAxisPrep.is_left;
	_zAxis.steps_left = _zAxisPrep.steps_left;
	_zAxis.millis_per_step = _zAxisPrep.millis_per_step;
	_zAxis.position = _zAxisPrep.position;
	_zAxis.last_step = _zAxisPrep.last_step;

	_decceleration_steps = _decceleration_stepsPrep;
	_speed_multiplier = _speed_multiplierPrep;
    _fastest_axis = _fastest_axisPrep;

	//enabling motors
	//X
	if(_xAxis.is_left)
		digitalWrite(_xAxis.cw_pin, HIGH);
	else
		digitalWrite(_xAxis.cw_pin, LOW);
	digitalWrite(_xAxis.enable_pin, HIGH);
	//Y
	if(_yAxis.is_left)
		digitalWrite(_yAxis.cw_pin, HIGH);
	else
		digitalWrite(_yAxis.cw_pin, LOW);
	digitalWrite(_yAxis.enable_pin, HIGH);
	//Z
	if(_zAxis.enabled)
	{
		if(_zAxis.is_left)
			digitalWrite(_zAxis.cw_pin, HIGH);
		else
			digitalWrite(_zAxis.cw_pin, LOW);
		digitalWrite(_zAxis.enable_pin, HIGH);
	}

	_status = CNCProcessing;
    _prepaired = 0;
	_last_millis = micros();

	//Serial.print("ES ");
	//Serial.println(_fastest_axis->steps_left);
}

void DACNCRouter::_traingleLineTo(double x, double y, double right_x, double right_y)
{
    
}

//proceed controller tick for axis
void DACNCRouter::_axisAct(DACNCAxis *axis, double dMls)
{
    if(axis->steps_left>0)
    {
		/*digitalWrite(axis->enable_pin, HIGH); */
//        double millis_current = mls - axis->last_step;
//        
//        if(axis->last_step == 0) millis_current = axis->millis_per_step;
        
        axis->last_step+=dMls;
//        Serial.print("LS ");
//        Serial.println(axis->last_step);
        
	 /*   if(axis->is_left)
			digitalWrite(axis->cw_pin, HIGH);
		else
			digitalWrite(axis->cw_pin, LOW);  */
        
//        if(millis_current>=axis->millis_per_step && !axis->pin_level)
        digitalWrite(axis->step_pin, HIGH);
        if(axis->last_step>=axis->millis_per_step) //making step
        {
//			Serial.print("E");
//            Serial.println(axis->steps_left);
//            Serial.print(" ");
//            Serial.print(axis->millis_per_step);
            
            //digitalWrite(axis->step_pin, HIGH);
			//axis->pin_level = 1;
//            axis->last_step+=millis_current;
            axis->last_step-=axis->millis_per_step;
		   // if(axis->is_left)
		   //     axis->position -= 1.0/axis->steps_per_mm;
		   // else
		   //     axis->position += 1.0/axis->steps_per_mm;
            
            digitalWrite(axis->step_pin, LOW);
            //axis->pin_level = 0;
			axis->steps_left--;
		   /* if(axis->end_pin1) //endstop
            {
                if(digitalRead(axis->end_pin1)) axis->steps_left = 0;
            }   */
            
            //setting accel
			if(_accelerated && axis == _fastest_axis) {
                if(axis->steps_left < _decceleration_steps)
                {
					_speed_multiplier-=_speed_increment;
                    if(_speed_multiplier<0.05) _speed_multiplier = 0.05;
                }
                else if(_speed_multiplier < 1.0) {
                    _speed_multiplier+=_speed_increment;
                    if(_speed_multiplier > 1.0)
                        _speed_multiplier = 1.0;
                }
            }

//            Serial.print(" ");
//            Serial.println(axis->last_step);
        }
//        else if(millis_current>=axis->millis_per_step/2 && axis->pin_level)
//        else if(axis->last_step>=axis->millis_per_step/2 && axis->pin_level)
//        {
//            digitalWrite(axis->step_pin, LOW);
//            axis->pin_level = 0;
//            axis->steps_left--;
//        }
    }
    else digitalWrite(axis->enable_pin, HIGH);
}
