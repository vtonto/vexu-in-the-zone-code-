void eight_one_blue()
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
  jack_choose = false;
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
	pid_move(2000);
	Target_Jack = JackIniD + deltaJack;
	claw_loose = true;
	delay(100);
	Target_Jack = JackIniU + deltaJack;
	Target_Arm = ARMBACK;

	check();//**********************ALPHA RED first check turn*************************//
  delay(150);

	//*******************************ALPHA RED SECOND CATCH****************************//2
	claw_hold=true;
	startTask(second_peri);
	pid_move(420);
	claw_hold = true;
	spin_forward = true;
	delay(140);
	stopTask(second_peri);
	//SensorValue[gyro]=0;
	check();

	//********************ALPHA RED COME BACK TO FETCH ANOTHER FOUR*******************//3(1)
	startTask(second_drop);
	pid_move(-1070);
	stopTask(second_drop);
	Target_Jack = JackPre + deltaJack;
	claw_hold = true;
	startTask(third_peri);
	pid_turn(-952);
	//base_set(50,50);
	//delay(70);
	//base_set(0,0);
	jack_choose = true;
	delay(200);//time to suck
	spin_backward = true;
	stopTask(third_peri);
	delay(230);//enough time to spin backward
	Target_Jack = JackTri + deltaJack;
	delay(250);
	claw_loose = true;
	delay(70);//enough time to jack down

	//*******************************ALPHA RED REPEAT THE PROCESS**********************//4(2)
  Target_Jack = JackPreD;// 1624;//////////////////////////4
  claw_loose = true;
  delay(200);
  spin_forward = true;
  delay(350);
  claw_hold = true;
  delay(160);
  Target_Jack = JackPreD+200;// 1624;//////////////////////////4
  spin_backward = true;
  delay(250);
  Target_Jack = JackFour;// 1250;
  delay(200);
  claw_loose = true;
  delay(200);

	//*******************************ALPHA RED REPEAT THE PROCESS**********************//5(3)
  Target_Jack = JackPreD + deltaJack;// 1624;//////////////////////////5
  claw_loose = true;
  delay(300);
  spin_forward = true;
  delay(200);
  claw_hold = true;
  delay(250);
  Target_Jack = JackFive + deltaJack;// 1750;
  delay(70);
  spin_backward = true;
  delay(370);
  Target_Jack = JackHold + deltaJack;// 1500;
  delay(320);

	//*******************************ALPHA RED REPEAT THE PROCESS**********************//6(4)
  Target_Jack = JackPreD+100 + deltaJack;// 1764;//////////////////////////6
  claw_loose = true;
  delay(380);
  spin_forward = true;
  delay(100);
  Target_Jack = JackPreD + deltaJack;// 1624;
  delay(200);
  claw_hold = true;
  delay(200);
  Target_Jack = JackSix + deltaJack - 100;// 1950;
  delay(80);
  spin_backward = true;
  delay(310);
  Target_Jack = JackHold+100 + deltaJack;// 1600;
  delay(200);
  claw_loose = true;
  delay(230);

	//*******************************ALPHA RED REPEAT THE PROCESS**********************//7(5)
  Target_Jack = JackPreD+200 + deltaJack;// 1864;//////////////////////////7
  claw_loose = true;
  delay(340);
  spin_forward = true;
  delay(100);
  Target_Jack = JackPreD + deltaJack;// 1624;
  delay(100);
  claw_hold = true;
  delay(330);
  Target_Jack = JackSeven + deltaJack;//2050;
  delay(60);
  claw_hold = true;
  spin_backward = true;
  delay(330);
  Target_Jack = JackHold+200 + deltaJack;//1700;
  delay(200);
  claw_loose = true;
  delay(200);

  //*******************************ALPHA RED REPEAT THE PROCESS**********************//7(5)
  Target_Jack = JackPreD+300 + deltaJack;// 1864;//////////////////////////7
  claw_loose = true;
  delay(390);
  spin_forward = true;
  delay(100);
  Target_Jack = JackPreD + deltaJack;// 1624;
  delay(100);
  claw_hold = true;
  delay(350);
  Target_Jack = JackSeven + deltaJack+200;//2050;
  delay(120);
  claw_hold = true;
  spin_backward = true;
  delay(330);
  Target_Jack = JackHold+200 + deltaJack;//1700;
  delay(200);
  claw_loose = true;
  delay(200);

	//*******************************ALPHA RED REPEAT THE PROCESS**********************//8(6)
  Target_Jack = JackPreD+680;// 2064;//////////////////////////8
  claw_loose = true;
  delay(250);
  claw_loose = true;
  spin_forward = true;
  delay(300);
  Target_Jack = JackPreD;// 1624;
  delay(350);
  claw_hold = true;
  delay(100);
  startTask(eighth_drop);
  jack_choose = false;
	pid_turn(948);

	//************************NOW IT RETURN TO THE INITIAL POSITION*********************//
	pid_move(-1350);
	stopTask(eighth_drop);//*****************now jack task is turned off*********************//
	base_set(-40,-40);
	delay(500);
	base_set(0,0);

	//********ALPHA RED a series of turns to deliver the first big point thing*********//
	pid_turn(-410);
	pid_move(-1370);//move backside til the center of the zone
	pid_turn(-930);//face up to the zone
  //ArmKeep = false;
	JackKeep = true;//******************turn on the jack task again*********************//
	stopTask(PID_Claw);
	control_claw(0);
	base_set(127,127);
	delay(800);
	JackKeep = true;
	claw_loose = true;
	startTask(PID_Crank);
  spin_backward = true;
	base_set(100,100);
	delay(500);
	base_set(50,50);
	ArmKeep= true;
	Target_Arm = ARMFRONT;
	Target_Jack = Target_Jack + 100;
	delay(100);
	claw_loose = true;
	spin_forward = true;
	//delay(200);
	claw_loose = true;
	delay(800);
	base_set(0,0);
	control_claw(100);
	//Target_Jack = JackHigh + deltaJack;
	//delay(450);
	JackKeep = true;
	//stopTask(PID_Jack);
	stopTask(PID_Crank);
	control_crank(-30);
	spin_backward = spin_forward = false;
	//control_jack(90);
	Target_Jack = JackHigh + deltaJack + 540;
	delay(550);
	control_claw(30);
	startTask(PID_Crank);
	spin_backward =true;
	delay(300);
	startTask(zone_back);
	claw_hold = claw_loose = false;
	base_set(-70,-70);
	delay(150);
	base_set(0,0);
	pid_move(-850);
	control_claw(0);
	claw_hold= true;
	startTask(PID_Claw);
	spin_backward = true;
	stopTask(zone_back);
	Target_Jack = JackDown + deltaJack;
//******************turn off the jack task again*******************//
	base_set(0,0);
	base_set(30,30);
	delay(600);
	base_set(0,0);
	JackKeep = false;
	stopTask(PID_Crank);
	control_crank(-30);
	//***************************ALPHA RED to check the position************************//
	spin_forward = spin_backward = false;
	pid_move(-230);
	ArmKeep = false;
	//***************************ALPHA RED MAKE RIGHT THE POSITION**********************//
	pid_turn(950);
	pid_move(-1085);
	base_set(-127,-40);
	delay(900);
	base_set(0,0);

	//******************************ALPHA RED SECOND START******************************//
	JackKeep = true;
	check_switch =true;//choose ALPHA RED long side
	Target_Jack = JackNine + deltaJack;
	startTask(PID_Crank);
	//spin_forward =true;
	pid_move(140);
	pid_turn(945);
	SensorValue[gyro]=0;
	ArmKeep = true;
	Target_Arm=ARMFRONT ;
	delay(100);
  spin_forward =true;
  Target_Jack = JackIni + deltaJack+80;
	pid_move(1750);
	Target_Jack = JackIniD + deltaJack;
	claw_loose = true;
	delay(100);
	Target_Jack = JackIniU + deltaJack;
	Target_Arm = ARMBACK;
	delay(300);
	//while(SensorValue[push_angle]>2030)
	//{
	//	delay(5);
	//}
	claw_hold=true;
	startTask(second_peri);
	spin_forward = true;
	base_set(60,-60);
	delay(50);
	base_set(0,0);
	spin_forward = true;
	claw_hold = true;
	pid_move(380);
	delay(180);
	stopTask(second_peri);
	startTask(second_drop);
	//check();
	//********************ALPHA RED COME BACK TO FETCH ANOTHER FOUR*******************//3(1)
	pid_move(-2700);
	stopTask(second_drop);
	claw_loose = true;
	stopTask(second_drop);
	pid_turn(450);
	pid_move(-760);
  pid_turn(450*2+290);
  base_set(70,70);
	Target_Arm=ARMFRONT;
	Target_Jack= JackIniD + deltaJack;
	delay(500);
	base_set(-100,-100);
	delay(600);
	Target_Arm=ARMBACK;
	base_set(0,0);
	pid_turn(250);
	base_set(-127,-127);
	delay(1000);
	base_set(0,0);
  pid_turn(450*2);
	stopTask(PID_Claw);
	stopTask(PID_Crank);
	stopTask(PID_Jack);
	stopTask(PID_Arm);
	//stopTask(safe_guard);
	control_arm(0);
	control_claw(0);
	control_jack(0);
  control_crank(0);
}
