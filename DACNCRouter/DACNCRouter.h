/*

*/

// ensure this library description is only included once
#ifndef Stepper_h
#define Stepper_h

#define _one_step_accel 0.07

typedef enum
{
    CNCUninitialized,
    CNCIdle,
    CNCProcessing,
    CNCError
} DACNCStatus;

typedef struct
{
    int enabled;
    int steps;
	int step_pin;
    int cw_pin;
    int enable_pin;
    int end_pin1;
    int left_limit_pin;
    int right_limit_pin;
    volatile double steps_per_mm;
    double length;
    double position;
    
    double last_step;
	double millis_per_step;
	double final_position;
    int is_left;
    long steps_left;
    int pin_level;
} DACNCAxis;

typedef struct
{
    int enabled;
    int on_pin;
    double radius;
} DACNCInstrument;

// library interface description
class DACNCRouter
{
public:
    DACNCRouter();
// setters for router axis parameters
    void setAxisX(int motor_steps_number, int motor_step_pin, int motor_cw_pin, int motor_enable_pin, double mm_per_step, double axis_length_mm, int left_limit_pin, int right_limit_pin);
    void setAxisY(int motor_steps_number, int motor_step_pin, int motor_cw_pin, int motor_enable_pin, double mm_per_step, double axis_length_mm, int left_limit_pin, int right_limit_pin);
    void setAxisZ(int motor_steps_number, int motor_step_pin, int motor_cw_pin, int motor_enable_pin, double mm_per_step, double axis_length_mm, int left_limit_pin, int right_limit_pin);
// setter for instrument
    void setWorkingInstrument(int instrument_on_pin, double instrument_radius);
// calls every program runloop step
    void act();
//getter for router status (see DACNCStatus enum)
    DACNCStatus status();
//getter for router version
	int version();
//getter for prepaired fro next step
	int prepaired();
    
	void setCurrentPosition(double x, double y, double z);
	void prepareLineTo(double x, double y, double z, double f, int next=0, double x1=.0, double y1=.0, double z1=.0, double f1=-1.);
	void lineTo(double x, double y, double z, int next=0, double x1=.0, double y1=.0, double z1=.0, double f1=-1.);
    void setNextPoint(double x, double y, double z);
    void lineToNextPoint();
    void setInstrumentPower(int p);
    void setSpeed(double s);
    void setMaxStartSpeed(double s);
    
private:
    void _setAxis(DACNCAxis *axis, int motor_steps_number, int motor_step_pin, int motor_cw_pin, int motor_enable_pin, double mm_per_step, double axis_length_mm, int left_limit_pin, int right_limit_pin);
//start linear movement
    void _lineTo(double x, double y, double z, int next=0, double x1=.0, double y1=.0, double z1=.0, double f1=-1.);
    void _traingleLineTo(double x, double y, double right_x, double right_y);
//proceed controller tick for axis
    void _axisAct(DACNCAxis *axis, double dMls);

//router physical parameters
    DACNCAxis _zeroAxis;
    DACNCAxis _xAxis;
    DACNCAxis _yAxis;
	DACNCAxis _zAxis;

	DACNCAxis _xAxisPrep;
	DACNCAxis _yAxisPrep;
	DACNCAxis _zAxisPrep;
    
    DACNCInstrument _instrument;
    
//router logical parameters
	volatile DACNCStatus _status;
    double _delay;
    double _epsilon;
    unsigned long _last_millis;
    double _max_start_speed;
    double _speed_increment;
	double _speed_multiplier;
	double _speed_multiplierPrep;
    int _accelerated;
    double speed;
	DACNCAxis *_fastest_axis;
	DACNCAxis *_fastest_axisPrep;
	long _decceleration_steps;
	long _decceleration_stepsPrep;
	int _prepaired;
};

#endif

