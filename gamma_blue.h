void short_break_blue();

void short_break_blue()
{
	//***************************INITIALIZE ALL THE STATE*****************************//
	SensorValue[gyro]=0;
	SensorValue[left_encoder]=SensorValue[right_encoder]=0;
	spin_forward = spin_backward = false;
	claw_hold = claw_loose = false;

	//***************************initialize the lift angle****************************//
	deltaJack = SensorValue[lift_angle] - JackStart;
	Target_Jack = JackDown + deltaJack;

	//*************************turn on the jack & arm task****************************//
	JackKeep = true;
	ArmKeep = true;
	BaseKeep = true;
	//***********************frist choose ALPHA BLUE short side***********************//
	check_switch = false;

	startTask(PID_Claw);
	startTask(PID_Crank);
	startTask(PID_Jack);
	startTask(PID_Arm);
	//startTask(safe_guard);


	//******************************ALPHA BLUE START NOW******************************//
	spin_forward = true;
	claw_hold =true;
	Target_Arm = ARMFRONT;
	//*********************************hold at the begining***************************//1
	Target_Jack = JackIni + deltaJack;
	pid_move(1990);
	Target_Jack = JackIniD + deltaJack;
	claw_loose = true;
	delay(100);
	Target_Jack = JackIniU + deltaJack;
	Target_Arm = ARMBACK;

	check();//**********************ALPHA RED first check turn*************************//
  delay(180);
	claw_hold=true;
	//*******************************ALPHA RED SECOND CATCH****************************//2
	startTask(second_peri);
	pid_move(430);
	SensorValue[gyro]=0;
	check();//**********************ALPHA RED first check turn*************************//
  delay(80);
  stopTask(second_peri);
  Target_Jack = Jack_Two + deltaJack;   //second lift up ready
  delay(50);
	spin_backward=true;
	delay(400);
	Target_Jack = Jack_TwoD + deltaJack;  //second lift down to lose
	delay(150);
	claw_loose=true;
	delay(100);

  startTask(three_peri);
  SensorValue[gyro]=0;
	pid_move(500);
	check();//**********************ALPHA RED first check turn*************************//3
	delay(80);
	stopTask(three_peri);
  Target_Jack = Jack_Three + deltaJack + 150;//second lift up ready
  delay(210);
	spin_backward=true;
	delay(370);
	Target_Jack = Jack_ThreeD + deltaJack;  //second lift down to lose
	delay(150);
	claw_loose=true;
	delay(100);

	startTask(four_peri);
	delay(530);
  SensorValue[gyro]=0;
	pid_move(400);
	check();//**********************ALPHA RED first check turn*************************//
	delay(200);
	stopTask(four_peri);
	startTask(four_drop);
	SensorValue[gyro]=0;
  check();//**********************ALPHA RED first check turn*************************//
  base_set(-50,50);
  delay(50);
  base_set(0,0);
  pid_move(-3320);
  stopTask(four_drop);
  base_set(-35,-35);
	delay(400);
	base_set(0,0);

	//********ALPHA RED a series of turns to deliver the first big point thing*********//
	pid_turn(470);
	pid_move(-1280);//move backside til the center of the zone
	pid_turn(920);//face up to the zone
	ArmKeep = false;
	base_set(127,127);
	JackKeep = true;
	Target_Jack = 1663+300;
	spin_backward = true;
	claw_loose = true;
	delay(600);
	delay(400);
	base_set(70,70);
   //******************turn on the jack task again*********************//
	//Target_Jack = 1663+100;
	spin_backward = true;
	claw_loose = true;
	delay(200);
	spin_backward = true;
	ArmKeep = true;
	Target_Arm = ARMFRONT;
	delay(800);
	base_set(0,0);
	spin_backward = true;
	startTask(zone_back_beta);
	claw_hold = claw_loose = false;
	pid_move(-950);
	stopTask(zone_back);
	Target_Jack = JackDown + deltaJack;
	JackKeep = false;//******************turn off the jack task again*******************//
	delay(10);
	base_set(0,0);
	pid_turn(-470);
	base_set(-127,-127);
	delay(1200);
	pid_turn(450);
	base_set(-127,-127);
	delay(2500);
	base_set(0,0);
	stopTask(PID_Claw);
	stopTask(PID_Crank);
	stopTask(PID_Jack);
	stopTask(PID_Arm);
	//stopTask(safe_guard);
}
