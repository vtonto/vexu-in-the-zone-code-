void short_four_three_blue();

void short_four_three_blue()
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
	check_switch = true;

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
	pid_move(425);
	SensorValue[gyro]=0;
	check();//**********************ALPHA RED first check turn*************************//
  delay(80);
  stopTask(second_peri);
  Target_Jack = Jack_Two + deltaJack;   //second lift up ready
	spin_backward=true;
	delay(400);
	Target_Jack = Jack_TwoD + deltaJack;  //second lift down to lose
	delay(200);
	claw_loose=true;
	delay(100);

  startTask(three_peri);
  SensorValue[gyro]=0;
	pid_move(440);
	check();//**********************ALPHA RED first check turn*************************//3
	delay(80);
	stopTask(three_peri);
  Target_Jack = Jack_Three + deltaJack;//second lift up ready
  delay(150);
	spin_backward=true;
	delay(380);
	Target_Jack = Jack_ThreeD + deltaJack;  //second lift down to lose
	delay(180);
	claw_loose=true;
	delay(100);

	startTask(four_peri);
	delay(500);
  SensorValue[gyro]=0;
	pid_move(415);
	check();//**********************ALPHA RED first check turn*************************//
	delay(200);
	stopTask(four_peri);
	startTask(four_drop);
	SensorValue[gyro]=0;
  check();//**********************ALPHA RED first check turn*************************//
  pid_move(-3320);
  stopTask(four_drop);
  base_set(-35,-35);
	delay(400);
	base_set(0,0);

	//********ALPHA RED a series of turns to deliver the first big point thing*********//
	pid_turn(-480);
	pid_move(-1350);//move backside til the center of the zone
	pid_turn(-910);//face up to the zone
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
	pid_turn(930);
	pid_move(-1050);
	base_set(-127,-50);
	delay(500);
	base_set(-70,-70);
	delay(700);
	base_set(0,0);
	delay(20);

	pid_move(130);
	JackKeep = true;
	check_switch =false;//choose ALPHA RED long side
	Target_Jack = JackIni + deltaJack;
	spin_forward = true;
	pid_turn(950);
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
	pid_move(415);
	check();//**********************ALPHA RED first check turn*************************//
  delay(150);
  stopTask(second_peri);
  Target_Jack = Jack_Two + deltaJack;   //second lift up ready
	spin_backward=true;
	delay(400);
	Target_Jack = Jack_TwoD + deltaJack;  //second lift down to lose
	delay(200);
	claw_loose=true;
	delay(100);
	base_set(50,-50);
	delay(70);
	base_set(0,0);
	startTask(three_peri);
	pid_move(440);
	check();//**********************ALPHA RED first check turn*************************//3
	delay(200);
	stopTask(three_peri);

  Target_Jack = Jack_Three + deltaJack;   //second lift up ready
	spin_backward=true;
	delay(500);
	Target_Jack = Jack_ThreeD + deltaJack;  //second lift down to lose
	delay(200);
	claw_loose=true;
	delay(100);
	//base_set(60,-60);
	//delay(70);
	//base_set(0,0);

	startTask(four_peri);
	delay(550);
  SensorValue[gyro]=0;
	pid_move(410);
	check();//**********************ALPHA RED first check turn*************************//
	delay(250);
	stopTask(four_peri);
	startTask(four_drop);
	SensorValue[gyro]=0;
  check();//**********************ALPHA RED first check turn*************************//
  pid_move(-3370);
  stopTask(four_drop);
	pid_turn(480);
	pid_move(-1700);
  pid_turn(450*2);
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
