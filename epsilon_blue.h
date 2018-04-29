void long_break_blue();

void long_break_blue()
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

	startTask(PID_Claw);
	startTask(PID_Crank);
	startTask(PID_Jack);
	//startTask(PID_Arm);

	claw_hold = true;
  spin_backward = true;
	Target_Arm = ARMBACK;
  Target_Jack = JACKH+deltaJack;
  startTask(pre_score);
  pid_move(470);
  claw_loose = true;
  Target_Jack = JACKH+deltaJack + 200;
  delay(200);
  stopTask(pre_score);
  spin_backward = true;
  pid_move(-200);
  pid_turn(-930);
  stopTask(PID_Crank);
  control_crank(-30);
  Target_Jack = JackDown +deltaJack;
  pid_move(-1490);
  pid_turn(-700);
  base_set(-127,-127);
  delay(2000);
  base_set(0,0);
  pid_turn(60);
  pid_move(3000);
  pid_turn(980);
  base_set(-80,-80);
  delay(700);
  pid_move(130);
  pid_turn(935);

  startTask(PID_Arm);
  startTask(PID_Crank);
	SensorValue[gyro]=0;
	spin_forward = true;
	Target_Arm=ARMFRONT;
	claw_hold =true;
	check_switch = false;
	Target_Jack = JackIni + deltaJack;
	//*********************************hold at the begining***************************//1
	pid_move(1800);
	Target_Arm= ARMBACK;
  delay(150);
  base_set(50,-50);
  delay(50);
  base_set(0,0);
	//*******************************ALPHA RED SECOND CATCH****************************//2
	pid_move(370);
	claw_hold=true;
	Target_Jack = JackDown + deltaJack;
  //**********************ALPHA RED first check turn*************************//
  delay(700);
  base_set(40,-70);
  delay(90);
  base_set(0,0);
  stopTask(second_peri);
  startTask(second_drop);
  // Target_Jack = Jack_Two + deltaJack;   //second lift up ready
	//spin_backward=true;
	//delay(400);
	//Target_Jack = Jack_TwoD + deltaJack;  //second lift down to lose
	//delay(200);
	//claw_loose=true;
	//delay(100);
	pid_move(-2200);
  stopTask(second_drop);
	base_set(-35,-35);
	delay(500);
	base_set(0,0);

	//********ALPHA RED a series of turns to deliver the first big point thing*********//
	pid_turn(480);
	pid_move(-1420);//move backside til the center of the zone
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
	pid_move(-900);
	stopTask(zone_back);
	Target_Jack = JackDown + deltaJack;
	JackKeep = false;//******************turn off the jack task again*******************//
	delay(10);
	base_set(0,0);
	base_set(35,35);
	delay(500);
	base_set(0,0);
	delay(10);
	//***************************ALPHA RED to check the position************************//
	pid_move(-240);
	pid_turn(-930);
	pid_move(-1050);
	base_set(-50,-127);
	delay(500);
	base_set(-70,-70);
	delay(700);
	base_set(0,0);
	delay(20);

	pid_move(120);
	JackKeep = true;
	check_switch =true;//choose ALPHA RED long side
	Target_Jack = JackIni + deltaJack;
	spin_forward = true;
	pid_turn(-960);
	SensorValue[gyro]=0;
	Target_Arm=ARMFRONT;
	claw_hold =true;
	//*********************************hold at the begining***************************//1
	pid_move(1740);
	Target_Arm= ARMBACK;
  delay(150);
  check();
	claw_hold=true;
	//*******************************ALPHA RED SECOND CATCH****************************//2
	startTask(second_peri);
	pid_move(420);
	check();//**********************ALPHA RED first check turn*************************//
  delay(150);
  startTask(second_drop);
  stopTask(second_peri);
  pid_move(-2700);
  stopTask(four_drop);
	pid_turn(-480);
	pid_move(-800);
  pid_turn(-450*2);
  base_set(70,70);
	Target_Arm=ARMFRONT;
	Target_Jack= JackIniD + deltaJack;
	delay(900);
	base_set(-127,-127);
	delay(500);
	base_set(0,0);
	pid_turn(-480);
	Target_Arm=ARMBACK;
  base_set(-127,-127);
  delay(1300);
	base_set(0,0);
  pid_turn(450*3);
	base_set(-127,-127);
	delay(3500);
	//Target_Arm=ARMBACK;
	//base_set(0,0);
	//pid_turn(-350);
	//base_set(-127,-127);
	//delay(500);
	//base_set(0,0);
 // pid_turn(450*3);
	//base_set(-127,-127);
	//delay(800);
	//delay(700);



	base_set(0,0);
	stopTask(PID_Claw);
	stopTask(PID_Crank);
	stopTask(PID_Jack);
	stopTask(PID_Arm);
	//stopTask(safe_guard);




}
