#define MOVE   0
#define TURN   1
#define JACK   2
#define ARM    3
#define CRANK  4
#define CLAW   5

#define PIDLIB_LUT_SIZE         128
#define PIDLIB_LUT_FACTOR       20.0
#define PIDLIB_LUT_OFFSET       10
#define ARMFRONTPOS             3430
#define ARMBACKPOS              1600



float move,turn,push,lift,grab,spin;
int arm_signal;
int spin_signal;

void base_set(int leftPower, int rightPower);
void control_arm(int power);
void control_jack(int power);
void control_crank(int power);
void control_claw(int power);
int linear(int ch_value);
task fetch_value();



void user_control()
{


	startTask(fetch_value);
	while(true)
	{
		base_set(move+turn, move-turn);
		control_arm(push*127);
		control_claw(grab*127);
		control_crank(spin*127);
		control_jack(lift*127);
	}
}

task fetch_value()
{
	while (true)
	{
	  arm_signal = SensorValue[push_angle] > ARMFRONTPOS ? 1 : ( SensorValue[push_angle] < ARMBACKPOS ? (-1) : 0);
	  spin_signal = SensorValue[spin_angle] > 3300 ? 1 : ( SensorValue[spin_angle] < 1100 ? (-1) : 0);
	  move  = linear( abs(vexRT[Ch3]) > 30 ? vexRT[Ch3] : 0 );
		turn  = linear( abs(vexRT[Ch4]) > 30 ? vexRT[Ch4] : 0 );
		lift  = (-vexRT[Btn5D] + vexRT[Btn5U]);//*middle_lift ;  //U upward, D downward
		push  = -vexRT[Btn8L] + vexRT[Btn8R] + 0.21*arm_signal ;  //R forward, L withdraw
		grab  = -vexRT[Btn6U] + vexRT[Btn6D]+0.3*(1-vexRT[Btn6U])*(1-vexRT[Btn6D]);  //D catch, U loose
		spin  = -vexRT[Btn8D]*(1+spin_signal) + vexRT[Btn8U]*(1-spin_signal)+
		        0.15*spin_signal*(1-vexRT[Btn8U])+0.15*spin_signal*vexRT[Btn8U] ;  //U forward, D withdraw
	}
}


void base_set(int leftPower, int rightPower)
{
	motor[left_front_wheel] = leftPower;
	motor[left_back_wheel] = leftPower;
	motor[right_front_wheel] = rightPower;
	motor[right_back_wheel] = rightPower;
}

void control_arm(int power)
{
	motor[arm_reach]=power;
}

void control_jack(int power)
{
	motor[jack_left]=motor[jack_right]=power;
}

void control_crank(int power)
{
	motor[crank_left]=motor[crank_right]=power;
}

void control_claw(int power)
{
	motor[catching]=power;
}

int linear(int ch_value)
{
	int  linear;
	float x;

	x = pow( PIDLIB_LUT_FACTOR, (float)(abs(ch_value)) / (float)(PIDLIB_LUT_SIZE-1) );
	if(abs(ch_value) >= (PIDLIB_LUT_OFFSET/2))
		linear = (((x - 1.0) / (PIDLIB_LUT_FACTOR - 1.0)) * (PIDLIB_LUT_SIZE-1-PIDLIB_LUT_OFFSET)) + PIDLIB_LUT_OFFSET;
	else
		linear = ch_value* 2;
	return sgn(ch_value)*linear;
}
