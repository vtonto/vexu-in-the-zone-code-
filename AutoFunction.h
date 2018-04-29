#ifndef __AUTOFUNCTION_H
#define __AUTOFUNCTION_H

#define JACKH  2000

#include "HandFunction.h"

#define ARMBACK     1300
#define ARMMID      1400
#define ARMFRONT    3680
#define ARMSTART    1110

#define JackReady   1530
#define JackDown    1070
#define JackStart   1070
#define JackIni     1670
#define JackIniD    1600
#define JackIniU    1840
#define JackTwo     1350
#define JackTwoD    1200


#define Jack_Two     1300
#define Jack_TwoD    1096
#define Jack_Three   1500
#define Jack_ThreeD  1311
#define Jack_Four    1700
#define Jack_FourD   1356



#define JackPre     1724
#define JackPreD    1624
#define JackTri     1000
#define JackFour    1250
#define JackFive    1750
#define JackSix     1950
#define JackSeven   2050
#define JackEight   2100
#define JackHold    1500
#define JackHigh    2380
#define JackNine    1790

int deltaJack;



struct PID
{
	int kILimit;
	int error;
	int lastError;
	float integral;
	float derivative;
	bool timerBool;
	float PowerOut;
};

#define KpMoveM    0.445
#define KiMoveM    0.005
#define KdMoveM    2.06

#define KpMoveL    0.332
#define KiMoveL    0.000003
#define KdMoveL    0.972

#define KpMoveS    0.535
#define KiMoveS    0.04
#define KdMoveS    0.66

#define KpMoveSS   3.5
#define KiMoveSS   0.0
#define KdMoveSS   0.0

#define KpTurn    0.57//0.52
#define KiTurn    1.08//0.85
#define KdTurn    3.72//3.72

#define Kp     2.48
#define Ki     0.13
#define Kd     0.90
#define KpJackL    0.37//0.333
#define KiJackL    0.01//0.0
#define KdJackL    0.5//0.0

#define KpJackS    0.513
#define KiJackS    0.032//0.0
#define KdJackS    0.01//0.0

#define KpArm     1.60
#define KiArm     0.6
#define KdArm     0.8

PID Argo[5];
int Target_Arm,Target_Jack;
bool spin_forward,spin_backward;
bool claw_hold,claw_loose;
bool Jack_tar_over;
bool Arm_tar_over;
bool move_tar_over;
bool turn_tar_over;
bool JackKeep;
bool check_switch;
bool ArmKeep;
bool BaseKeep;
bool jack_choose;

bool bcrank=false;
int a;
void PIDInit(struct PID *controller);
void pid_move(int target);
void pid_turn(int target);
task PID_Arm();
task PID_Jack();
task PID_Crank();
task PID_Claw();
void check();

void pid_move(int target)
{
	PIDInit(&Argo[MOVE]);
	move_tar_over=false;
	SensorValue[left_encoder]=SensorValue[right_encoder]=0;
	clearTimer(T3);
	clearTimer(T4);
	while( time1[T3]<280 && BaseKeep)
	{
		Argo[MOVE].error = target-(SensorValue[left_encoder]+SensorValue[right_encoder])/2;
		Argo[MOVE].integral += Argo[MOVE].error;
		Argo[MOVE].derivative = Argo[MOVE].error - Argo[MOVE].lastError;
		Argo[MOVE].lastError = Argo[MOVE].error;

		if(Argo[MOVE].integral > Argo[MOVE].kILimit)
			Argo[MOVE].integral = Argo[MOVE].kILimit;

		if(Argo[MOVE].integral < -Argo[MOVE].kILimit)
			Argo[MOVE].integral = -Argo[MOVE].kILimit;

		if(abs(Argo[MOVE].error)<100 && abs(	Argo[MOVE].derivative)<2  )
		{
			move_tar_over=true;
		}

		if(abs(target)>1000)
		{
			Argo[MOVE].PowerOut= KpMoveL*Argo[MOVE].error+KiMoveL*Argo[MOVE].integral+KdMoveL*Argo[MOVE].derivative;
		}
		else if(abs(target)>200)
		{
			Argo[MOVE].PowerOut= KpMoveM*Argo[MOVE].error+KiMoveM*Argo[MOVE].integral+KdMoveM*Argo[MOVE].derivative;
		}
		else if(abs(target)>100)
		{
			Argo[MOVE].PowerOut= KpMoveS*Argo[MOVE].error+KiMoveS*Argo[MOVE].integral+KdMoveS*Argo[MOVE].derivative;
		}
		else
		{
			Argo[MOVE].PowerOut= KpMoveSS*Argo[MOVE].error+KiMoveSS*Argo[MOVE].integral+KdMoveSS*Argo[MOVE].derivative;
		}
		base_set(linear( Argo[MOVE].PowerOut),linear(Argo[MOVE].PowerOut));
		if( abs(Argo[MOVE].error) < 30 )
		{
			Argo[MOVE].timerBool = false;
		}

		if( Argo[MOVE].timerBool )
		{
			clearTimer(T3);
		}
		datalogDataGroupStart();
		datalogAddValue(1,target);
		datalogAddValue(0,(SensorValue[left_encoder]+SensorValue[right_encoder])/2);
		  datalogDataGroupEnd();
		delay(20);
		if(time1[T4]>4000)
		{break;}
	}
	base_set(0,0);
}

void pid_turn(int target)
{

	PIDInit(&Argo[TURN]);
	SensorValue[gyro]=0;
	int initial = SensorValue[gyro];
	turn_tar_over=false;
	clearTimer(T4);
	clearTimer(T3);
	while( time1[T4]< 260 && BaseKeep)
	{
		Argo[TURN].error = target-SensorValue[gyro]+initial;
		Argo[TURN].integral += Argo[TURN].error;
		Argo[TURN].derivative = Argo[TURN].error - Argo[TURN].lastError;
		Argo[TURN].lastError = Argo[TURN].error;

		if(Argo[TURN].integral > Argo[TURN].kILimit)
			Argo[TURN].integral = Argo[TURN].kILimit;

		if(Argo[TURN].integral < -Argo[TURN].kILimit)
			Argo[TURN].integral = -Argo[TURN].kILimit;

		if(abs(Argo[TURN].error)<50 && abs(Argo[TURN].error-Argo[TURN].lastError)<1)
		{
			turn_tar_over=true;
		}

		Argo[TURN].PowerOut= KpTurn*Argo[TURN].error+KiTurn*Argo[TURN].integral+KdTurn*Argo[TURN].derivative;

		//base_set(-linear(Argo[TURN].PowerOut),linear(Argo[TURN].PowerOut));
		motor[left_front_wheel] = -linear(Argo[TURN].PowerOut);
		motor[left_back_wheel] = -linear(Argo[TURN].PowerOut);
		motor[right_front_wheel] = linear(Argo[TURN].PowerOut);
		motor[right_back_wheel] = linear(Argo[TURN].PowerOut);

		if( abs(Argo[TURN].error) < 40 )
		{
			Argo[TURN].timerBool = false;
		}
		if( Argo[TURN].timerBool )
		{
			clearTimer(T4);
		}
		//		datalogDataGroupStart();
		//datalogAddValue(1,target);
		//datalogAddValue(0,	SensorValue[gyro]);
		//  datalogDataGroupEnd();
		delay(20);
		if(time1[T3]>2000)
		{
			break;
		}
	}
	base_set(0,0);
}

task PID_Arm()
{
	Jack_tar_over=false;

	int Arm_Storage=ARMBACK;
	ArmKeep = true;
	while(true)
	{
		if(Arm_Storage!=Target_Arm)
		{
			Arm_tar_over=false;
			Arm_Storage=Target_Arm;
			PIDInit(&Argo[ARM]);

		}
		while( true )
		{
			/*********PID Calculator********/
			Argo[ARM].error =Target_Arm-SensorValue[push_angle];
			Argo[ARM].integral += Argo[ARM].error;
			Argo[ARM].derivative = Argo[ARM].error - Argo[ARM].lastError;
			Argo[ARM].lastError = Argo[ARM].error;
			/*********PID Calculator********/

			if(Argo[ARM].integral > Argo[ARM].kILimit)
				Argo[ARM].integral = Argo[ARM].kILimit;

			if(Argo[ARM].integral < -Argo[ARM].kILimit)
				Argo[ARM].integral = -Argo[ARM].kILimit;

			Argo[ARM].PowerOut = KpArm*Argo[ARM].error		+KiArm*Argo[ARM].integral		+KdArm*Argo[ARM].derivative;
			if(abs(Argo[ARM].error)<200 && abs(Argo[ARM].error-Argo[ARM].lastError)<2)
			{
				Argo[ARM].PowerOut = abs(Argo[ARM].PowerOut) > 30 ? (Argo[ARM].PowerOut >0 ? 30 : -30) : Argo[ARM].PowerOut;
			}

			if(abs(Argo[ARM].error)<200 && abs(Argo[JACK].error-Argo[ARM].lastError)<2)
			{
				Jack_tar_over=true;
			}

			//if(  abs(Argo[ARM].PowerOut)>60 && abs(Argo[ARM].derivative)<4)
			//{
			//  count ++;
			//}
			//if(count > 20)
			//{
			//  ArmKeep = false;
			//  stopAllTasks();
			//}
			//datalogDataGroupStart();
			//datalogAddValue(0,count);
			//datalogAddValue(1,Argo[ARM].derivative);
			//datalogAddValue(2,Argo[JACK].PowerOut);
			//   datalogDataGroupEnd();
			control_arm(Argo[ARM].PowerOut);
			delay(10);
		}
		control_arm(0);
	}
}


task PID_Crank()
{

	while(true)
	{
		if(spin_forward)
		{

			while( abs(SensorValue[spin_angle]-3500)>300 && !spin_backward )
			{
				motor[crank_left]=motor[crank_right]=127;
				delay(20);
			}
			motor[crank_left]=motor[crank_right]=100;
			delay(200);
			spin_forward =false;
			delay(20);
		}
		else if(spin_backward)
		{

			//while( time1(T2)<914 && !spin_forward)
			while( abs(SensorValue[spin_angle]-1000)>300 && !spin_forward )
			{
				motor[crank_left]=motor[crank_right]=-127;
				delay(20);
				if(SensorValue[spin_angle]<1400)
				{
					bcrank = true;
					//writeDebugStreamLine("here, %d" ,SensorValue[spin_angle]);
				}
			}
			motor[crank_left]=motor[crank_right]=-90;
			delay(200);
			spin_backward = false;
			delay(20);
		}
		else
		{
			control_crank(0);
		}
	}
}


task PID_Jack()
{
	int Jack_Storage=JackDown;
	JackKeep=true;
	//clearDebugStream();
	while(true)
	{
		if(Jack_Storage!=Target_Jack)
		{
			Jack_tar_over=false;
			Jack_Storage=Target_Jack;
			PIDInit(&Argo[JACK]);
		}
		a = abs(Target_Jack-SensorValue[lift_angle]);
		//writeDebugStreamLine("%d",a);
		while( JackKeep || !Jack_tar_over )
		{
			/*********PID Calculator*******/
			Argo[JACK].error =Target_Jack-(SensorValue[lift_angle]);
			Argo[JACK].integral += Argo[JACK].error;
			Argo[JACK].derivative = Argo[JACK].error - Argo[JACK].lastError;
			if(abs(Argo[JACK].error)<200 && abs(Argo[JACK].error-Argo[JACK].lastError)<2)
			{
				Jack_tar_over=true;
			}
			Argo[JACK].lastError = Argo[JACK].error;
			/*********PID Calculator*******/

			if(Argo[JACK].integral > Argo[JACK].kILimit*2)
				Argo[JACK].integral = Argo[JACK].kILimit*2;

			if(Argo[JACK].integral < -Argo[JACK].kILimit*2)
				Argo[JACK].integral = -Argo[JACK].kILimit*2;


			if(!jack_choose)
			{
				Argo[JACK].PowerOut=KpJackL*Argo[JACK].error
				+KiJackL*Argo[JACK].integral
				+KdJackL*Argo[JACK].derivative;
			}
			else
			{
				Argo[JACK].PowerOut=KpJackS*Argo[JACK].error
				+KiJackS*Argo[JACK].integral
				+KdJackS*Argo[JACK].derivative;
			}

			//if(a>460)
			//{
			//   Argo[JACK].PowerOut=linear(KpJackL*Argo[JACK].error
			//   +KiJackL*Argo[JACK].integral
			//   +KdJackL*Argo[JACK].derivative);
			//   }
			//   else
			//   {
			//   Argo[JACK].PowerOut=linear(KpJackS*Argo[JACK].error
			//   +KiJackS*Argo[JACK].integral
			//   +KdJackS*Argo[JACK].derivative);
			//   }
			datalogDataGroupStart();
			datalogAddValue(0,SensorValue[lift_angle]);
			datalogAddValue(1,Target_Jack);
			datalogDataGroupEnd();
			//if(abs(Argo[JACK].error)>100)
			//{
			//   Argo[JACK].PowerOut = sgn(Argo[JACK].PowerOut)*127;
			//}
			control_jack(Argo[JACK].PowerOut);
			delay(10);
		}
		control_jack(0);
	}
}


task PID_Claw()
{
	while(true)
	{
		if(claw_hold)
		{
			motor[catching]=127;
			delay(600);
			while(!claw_loose)
			{
				motor[catching]=80;
				delay(20);
			}
			claw_hold = false;
		}
		else if(claw_loose && !claw_hold)
		{
			motor[catching]=-127;
			delay(450);
			motor[catching]=0;
			delay(20);
			claw_loose = false;
		}
		else
		{ motor[catching]=0; }
	}
}

void check()
{
	PIDInit(&Argo[TURN]);
	turn_tar_over=false;
	clearTimer(T4);
	clearTimer(T3);
	int ini = SensorValue[gyro];
	if(abs(ini)>50)
	{
		//writeDebugStreamLine("gyro check");
		while( time1[T4]<200 )
		{
			Argo[TURN].error = SensorValue[gyro];
			Argo[TURN].integral += Argo[TURN].error;
			Argo[TURN].derivative = Argo[TURN].error - Argo[TURN].lastError;
			Argo[TURN].lastError = Argo[TURN].error;

			if(Argo[TURN].integral > Argo[TURN].kILimit)
				Argo[TURN].integral = Argo[TURN].kILimit;

			if(Argo[TURN].integral < -Argo[TURN].kILimit)
				Argo[TURN].integral = -Argo[TURN].kILimit;

			if(abs(Argo[TURN].error)>20)
			{
				clearTimer(T4);
			}

			if(abs(ini)<=32)
			{
				Argo[TURN].PowerOut= 2.48*Argo[TURN].error+0.13*Argo[TURN].integral+0.90*Argo[TURN].derivative;
			}
			else
			{
				Argo[TURN].PowerOut= 1.735*Argo[TURN].error+0.15*Argo[TURN].integral+0.96*Argo[TURN].derivative;
			}
			Argo[TURN].PowerOut = abs(Argo[TURN].PowerOut) > 90 ? sgn(Argo[TURN].PowerOut)*90 : Argo[TURN].PowerOut ;

			base_set(linear(Argo[TURN].PowerOut),-linear(Argo[TURN].PowerOut));
			if(time1[T3]>500)
				break;
			wait1Msec(10);
		}
		base_set(0,0);
	}

	else
	{
		if(check_switch)
		{
			base_set(-70,50);
			delay(30);
			base_set(-40,20);
			delay(30);
			base_set(0,0);
		}
		else
		{
			base_set(50,-70);
			delay(30);
			base_set(20,-40);
			delay(30);
			base_set(0,0);
		}
	}
}


void PIDInit(struct PID *controller)
{
	controller->error = controller->lastError = 0;
	controller->integral = controller->derivative = 0;
	controller->kILimit = 50;
	controller->timerBool = true;
	controller->PowerOut = 0;
}

#endif
