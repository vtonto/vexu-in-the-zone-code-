#include "AutoFunction.h"
task second_peri();
task second_drop();
task third_peri();
task eighth_drop();
task zone_back();
//task safe_guard();
task zone_back_beta();
//int sensor_base,sensor_arm;

//task safe_guard()
//{
//  while(true)
//  {
//    sensor_base = SensorValue[left_encoder];
//    sensor_arm = SensorValue[push_angle];
//    if(abs(Argo[ARM].PowerOut)>80 || abs(Argo[MOVE].PowerOut)>70)
//    {
//    	writeDebugStreamLine("Argo[ARM].PowerOut=%d,Argo[MOVE].PowerOut=%d",abs(Argo[ARM].PowerOut),abs(Argo[MOVE].PowerOut)>70);
//      delay(1200);
//      writeDebugStreamLine("deltaArm=%d,deltaMove=%d",abs(SensorValue[push_angle]-sensor_arm),abs(SensorValue[left_encoder]-sensor_base));
//      if(abs(SensorValue[push_angle]-sensor_arm)<10 && abs(Argo[ARM].PowerOut)>80)
//      {
//      	writeDebugStreamLine("in hrer");
//      	sensor_arm = SensorValue[push_angle];
//        writeDebugStreamLine("again,deltaArm=%d,deltaMove=%d",abs(SensorValue[push_angle]-sensor_arm),abs(SensorValue[left_encoder]-sensor_base));
//    	  writeDebugStreamLine("again,Argo[ARM].PowerOut=%d,Argo[MOVE].PowerOut=%d",abs(Argo[ARM].PowerOut),abs(Argo[MOVE].PowerOut)>70);
//          stopTask(PID_Arm);
//          stopAllTasks();
//          BaseKeep = false;
//          base_set(0,0);
//          control_arm(0);
//        }
//      }

//      //if(abs(SensorValue[left_encoder]-sensor_base)<10 && abs(Argo[MOVE].PowerOut)>70)
//      //{
//      //  sensor_base = SensorValue[left_encoder];
//      //	delay(1500);
//      //	if(abs(SensorValue[left_encoder]-sensor_base)<10 && abs(Argo[MOVE].PowerOut)>70)
//      //	{
//      //    stopTask(PID_Arm);
//      //    stopAllTasks();
//      //    BaseKeep = false;
//      //    base_set(0,0);
//      //    control_arm(0);
//      //  }
//      //}
//    }

//}


task third_peri()
{
	delay(600);
	spin_forward = true;
  delay(500);
  Target_Jack = JackPreD + deltaJack  ;
}

task second_peri()
{
	delay(180);
	Target_Jack = JackDown + deltaJack-100;
}

task second_drop()
{
	Target_Jack = JackTwo + deltaJack+50;
	delay(100);//second lift up ready
	spin_backward=true;
	delay(500);
	Target_Jack = JackTwoD + deltaJack-100;  //second lift down to lose
	delay(200);
	claw_loose=true;
	delay(100);
}


task eighth_drop()
{
  Target_Jack = JackEight+150;//2100;
  delay(240);
  spin_backward = true;
  delay(660);
  stopTask(PID_Crank);
  control_crank(-60);
  delay(100);
  control_crank(-30);
  delay(600);
  Target_Jack = JackHold - 400;//1800;
  delay(200);
  control_crank(0);
  delay(700);
  Target_Jack = SensorValue[lift_angle]-100;
  delay(200);
	//JackKeep = false;
}

task zone_back()
{
	delay(400);
	Target_Arm=ARMBACK;
	delay(600);
	Target_Jack = JackDown + deltaJack;
	spin_backward = true;
	JackKeep =true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////four + three part/////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
//task two_peri()
//{
//	delay(170);
//	Target_Jack = JackDown + deltaJack;
//}

//task two_drop()
//{
//  Target_Jack = Jack_Two + deltaJack;   //second lift up ready
//	spin_backward=true;
//	delay(400);
//	Target_Jack = Jack_TwoD + deltaJack;  //second lift down to lose
//	delay(200);
//	claw_loose=true;
//	delay(100);
//}

task three_peri()
{
	Target_Jack = Jack_Two + deltaJack;   //second lift up ready
	delay(280);
	spin_forward=true;
	claw_hold = true;
	delay(300);
	Target_Jack = JackDown + deltaJack;
}

task four_peri()
{
	Target_Jack = Jack_Three + deltaJack+100;   //second lift up ready
	delay(430);
	spin_forward=true;
	claw_hold = true;
	delay(500);
	Target_Jack = JackDown + deltaJack - 70;
}

//task seven_drop()
//{
//  Target_Jack = Jack_Three + deltaJack;   //second lift up ready
//	spin_backward=true;
//	delay(500);
//	Target_Jack = Jack_ThreeD + deltaJack;  //second lift down to lose
//	delay(200);
//	claw_loose=true;
//	delay(100);
//}

task three_drop()
{
	delay(300);
  Target_Jack = Jack_Three + deltaJack;   //second lift up ready
	spin_backward=true;
	delay(500);
	Target_Jack = Jack_ThreeD + deltaJack;  //second lift down to lose
	delay(250);
	claw_loose=true;
	delay(100);
}

task four_drop()
{
	delay(500);
  Target_Jack = Jack_Four + deltaJack; //second lift up ready
  delay(100);
	spin_backward=true;
	delay(500);
	Target_Jack = Jack_FourD + deltaJack;  //second lift down to lose
  delay(200);
	claw_loose=true;
	delay(100);
	JackKeep = false;
}

task zone_back_beta()
{
	delay(600);
	Target_Arm=ARMBACK;
	delay(600);
	Target_Jack = JackDown + deltaJack;
	spin_backward = true;
	JackKeep =true;
}

task pre_score()
{
	delay(490);
  spin_forward = true;
  delay(100);
}
