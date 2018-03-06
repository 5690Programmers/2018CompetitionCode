// Pick a Robot, or your motors will be sad!!!
//#define FRIDAY_
#define NIGEL_ //FRIDAY_
#define ROBOT_SPEED 0.65
#define CALIBRATE_STRAIGHT 0.6983
#define ENCODER_STEP 0.0099
#define NavX true

#include <iostream>
#include <memory>
#include <string>

#include <AHRS.h>

#include <Joystick.h>
#include <Encoder.h>
#include <XboxController.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Drive/DifferentialDrive.h>
#include <Timer.h>
#include <VictorSP.h>
#include <SpeedControllerGroup.h>
#include <Spark.h>
#include <PIDController.h>
#include <PIDOutput.h>
#include <CameraServer.h>

#include <WPILib.h>

#include "networktables/NetworkTableInstance.h"
using std::string;

class Robot: public frc::SampleRobot {
#ifdef FRIDAY_
	frc::Spark Right1 {1};
	frc::Spark Right2 {2};
	frc::Spark Left1 {3};
	frc::Spark Left2 {4};
#endif

#ifdef NIGEL_
	frc::VictorSP Right1 {1};
	frc::VictorSP Right2 {0};
	frc::VictorSP Left1 {3};
	frc::VictorSP Left2 {2};
#endif
//change values when wired
	frc::VictorSP IntakeL{8}; //see Intake controller group
	frc::VictorSP IntakeR{7}; //see Intake controller group
	frc::VictorSP Elevator{6}; //right and left triggers
	frc::VictorSP Climber{9}; //right bumper

	frc::DigitalInput ElevatorU {5};
	frc::DigitalInput ElevatorOU {4};

	frc::SpeedControllerGroup Intake{IntakeL, IntakeR}; //A, B, C, D buttons
	//driving speed controllers
	frc::SpeedControllerGroup GroupL{ Left1, Left2};
	frc::SpeedControllerGroup GroupR{ Right1, Right2};

	frc::DifferentialDrive myRobot {GroupL, GroupR};

	frc::XboxController Xbox { 0 };
	frc::Encoder EncoderR {0, 1, false, frc::Encoder::EncodingType::k4X};
	frc::Encoder EncoderL {2, 3, false, frc::Encoder::EncodingType::k4X};

	frc::AnalogInput DisStuff {3};

	frc::SendableChooser<std::string> chooser;
	frc::DigitalInput Limit {7};

	const std::string autoNameMiddle = "StartMiddle";
	const std::string autoNameLeft = "StartLeft";
	const std::string autoNameRight = "StartRight";
	const std::string autoNamePineapple = "StartPineapple";

	AHRS *ahrs;
	enum Height {DOWN,
		SWITCH,
		SCALE};


public:

	std::shared_ptr<NetworkTable> Table;//week zero network table

	//Gets the ultrasonic sensor's value
	int Ultrasensor();
	Height elevatorHeight = DOWN;

	Robot()
	{
		//Table = NetworkTable::GetTable("DataTable"); //UNNOTE WHEN NEEDED THIS IS NEEDED FOR COMPETITION
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		myRobot.SetExpiration(0.1);


		EncoderL.SetMaxPeriod(0.1);
		EncoderL.SetMinRate(10);
		EncoderL.SetDistancePerPulse(ENCODER_STEP); //may be .01
		EncoderL.SetReverseDirection(true);
		EncoderL.SetSamplesToAverage(7);

		EncoderR.SetMaxPeriod(0.1);
		EncoderR.SetMinRate(10);
		EncoderR.SetDistancePerPulse(ENCODER_STEP);//may be .01
		EncoderR.SetReverseDirection(false);
		EncoderR.SetSamplesToAverage(7);

		ahrs = new AHRS(SPI::Port::kMXP);

	}

	void RobotInit() {


		        try {
		            ahrs->EnableLogging(true);
		        } catch (std::exception& ex ) {
		            std::string err_string = "Error enabling object navX MXP:  ";
		            err_string += ex.what();
		            DriverStation::ReportError(err_string.c_str());
		        }
		chooser.AddDefault(autoNamePineapple,autoNamePineapple);
		chooser.AddDefault(autoNameMiddle, autoNameMiddle);
		chooser.AddObject(autoNameLeft, autoNameLeft);
		chooser.AddObject(autoNameRight,autoNameRight);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		frc::CameraServer::GetInstance()->StartAutomaticCapture();

	}

	void Autonomous() {
//comment what auto does what and make sure the top chooser matches
		auto autoSelected = chooser.GetSelected();
		// std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;
		std::string gameData;
				gameData=frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if(autoSelected==autoNamePineapple){
			PineappleTest();
		}
		if(autoSelected==autoNameMiddle){
			if(gameData[0] == 'L'){
				StartMiddleSwitchLeft();
			}
			else{
				StartMiddleSwitchRight();
			}
		}
		else if(autoSelected == autoNameLeft){
			//[]is start of start of scale and '' is start of switch
			if(gameData[0] =='L'){
				if(gameData[1] == 'L'){
					StartLeftSwitchLeftScaleLeft();
				}
				else{
					StartLeftSwitchLeftScaleRight();
				}
			}
			else if(gameData[0] =='R'){
				if(gameData[1] == 'L'){
					StartLeftSwitchRightScaleLeft();
				}
				else{
					StartLeftSwitchRightScaleRight();
				}
			}
		} //next section right side

		else if(autoSelected == autoNameRight){
			if(gameData[0] =='L'){
				if(gameData[1] == 'L'){
					StartRightSwitchLeftScaleLeft();
				}
				else{
					StartRightSwitchLeftScaleRight();
				}
			}
			else if(gameData[0] =='R'){
				if(gameData[1] == 'L'){
					StartRightSwitchRightScaleLeft();
				}
				else{
					StartRightSwitchRightScaleRight();
				}
			}
		}
		
	}

	void OperatorControl() override {
	{

		myRobot.SetSafetyEnabled(true);
		double deadzone = 0.2;
		double XboxY;
		double XboxX;
		int limitTiming(0);
		bool limit(false);
		bool tempLim1(false);
		bool tempLim2(false);
		int rumbleCount(0);
		//double Correction = (-EncoderL.GetDistance()*0.0104) - (EncoderR.GetDistance()*0.0104);

		while (IsOperatorControl() && IsEnabled()) {


			SmartDashboard::PutNumber( "Left DisNotRaw: ", (EncoderL.GetDistance()));//Left Motor Distance in inches  //*0.0104
			SmartDashboard::PutNumber( "Right DisNotRaw: ", (EncoderR.GetDistance()));//Right Motor Distance in inches  //*0.0104
			SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());//Is the NavX connected
		    SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());//What angle the robot is at
			SmartDashboard::PutNumber(  "IMU_TotalYaw",         ahrs->GetAngle());//What angle the robot is at for real
			SmartDashboard::PutBoolean( "IMU_IsMoving",         ahrs->IsMoving());//Is the robot moving?
			SmartDashboard::PutNumber(  "Velocity_X",           ahrs->GetVelocityX() );//How fast robot is in X direction
			SmartDashboard::PutNumber(  "Velocity_Y",           ahrs->GetVelocityY() );//How fast robot is in Y direction
	        SmartDashboard::PutNumber(  "Accel_X",       ahrs->GetRawAccelX() );//How far robot is in X direction
			SmartDashboard::PutNumber(  "Accel_Y",       ahrs->GetRawAccelY());//How far robot is in Y direction

//Tank with adjustments
			if(Xbox.GetX(XboxController::JoystickHand(0)) > deadzone || Xbox.GetX(XboxController::JoystickHand(0)) < -deadzone){
				XboxX = Xbox.GetX(XboxController::JoystickHand(0));

			}else{
				XboxX = 0;
			}

			if(-Xbox.GetY(XboxController::JoystickHand(0)) > deadzone || -Xbox.GetY(XboxController::JoystickHand(0)) < -deadzone){
				XboxY = -Xbox.GetY(XboxController::JoystickHand(0));
			}
			else{
				XboxY = 0;
			}
			//Button Commands}
/*	        if(Xbox.GetBackButton())
				myRobot.TankDrive(-1, -1);

			else if(Xbox.GetStartButton())
				myRobot.TankDrive(1, 1);

			else*/
//unnote when needed^
			if(elevatorHeight == SWITCH || elevatorHeight == SCALE){
				myRobot.ArcadeDrive( XboxY/2, XboxX/2, true);
				if(abs(ahrs->GetRawAccelX()) > 0.5 || abs(ahrs->GetRawAccelY()) > 0.5)
					rumbleCount = 100;
				if(rumbleCount > 0){
					Xbox.SetRumble(XboxController::kRightRumble, 0.5);
					Xbox.SetRumble(XboxController::kLeftRumble, 0.5);
				}
				else{
					Xbox.SetRumble(XboxController::kRightRumble, 0);
					Xbox.SetRumble(XboxController::kLeftRumble, 0);
				}

			}
			else{
				myRobot.ArcadeDrive( XboxY, XboxX/1.25, true);
				Xbox.SetRumble(XboxController::kRightRumble, 0);
				Xbox.SetRumble(XboxController::kLeftRumble, 0);
			}


	        //elevator
	        if (Xbox.GetY(XboxController::JoystickHand(1)) > 0.1){ //Limit switch change values when wired
	        	if (ElevatorU.Get() && ElevatorOU.Get()){
	        		Elevator.Set(-Xbox.GetY(XboxController::JoystickHand(1)));
	        	}
	        	else {
	        		Elevator.Set(0);
	        	}
	        }
	        else if (Xbox.GetY(XboxController::JoystickHand(1)) < -0.1){ //Limit switch change values when wired
	        	Elevator.Set(-Xbox.GetY(XboxController::JoystickHand(1)));
	        }
	        else{
	        	Elevator.Set(0);
	        }
	        //climber
	        if (Xbox.GetBumper(XboxController::JoystickHand(0))){
	        	Climber.Set(0.75);
	        }
	        else {
	        	Climber.Set(0);
	        }
	        //intake

	        frc::SmartDashboard::PutNumber("Triggered 0", Xbox.GetTriggerAxis(XboxController::JoystickHand(0)));
	        frc::SmartDashboard::PutNumber("Triggered 1", Xbox.GetTriggerAxis(XboxController::JoystickHand(1)));

	        if(Xbox.GetTriggerAxis(XboxController::JoystickHand(1)) > 0.2)
	        	Intake.Set(Xbox.GetTriggerAxis(XboxController::JoystickHand(1)));
	        else if(Xbox.GetTriggerAxis(XboxController::JoystickHand(0)) > 0.2)
	        	Intake.Set(-Xbox.GetTriggerAxis(XboxController::JoystickHand(0)));
	        else{
	        	Intake.Set(0);
	        }

			 //Variable Updates
	        if(Xbox.GetXButton())
	        	elevatorHeight = DOWN;
	        if(Xbox.GetBButton())
	        	elevatorHeight = SCALE;

	        //Correction = (-EncoderL.GetDistance()*0.0104) - (EncoderR.GetDistance()*0.0104);
	        if(!Limit.Get() && (limitTiming + 1)%2)
	        	tempLim1 = true;
	        else if(Limit.Get())
	        	tempLim1 = false;
	        if(!Limit.Get() && (limitTiming)%2)
	        	tempLim2 = true;
	        else if(Limit.Get())
	        	tempLim2 = false;
	        if((tempLim1 && !tempLim2) || (!tempLim1 && tempLim2))
	        	limit = true;
	        else
	        	limit = false;


//for this bit of logic, when we have the real robot we need to swap the Xbox.Get_Button to the elevator power,
//as the current state is for logic testing.
	        if(limit && limitTiming < 1 && elevatorHeight == DOWN){
	        	elevatorHeight = SWITCH;
	        	limitTiming = 200;
	        }
	        else if(limit && (limitTiming < 1) && (elevatorHeight == SWITCH) && (Xbox.GetYButton())){
	        	elevatorHeight = SCALE;
	        	limitTiming = 200;
	        }
	        else if(limit && (limitTiming < 1) && (elevatorHeight == SWITCH) && (Xbox.GetAButton())){
	        	elevatorHeight = DOWN;
	        	limitTiming = 200;
	        }
	        else if(limit && (limitTiming < 1) && (elevatorHeight == SCALE) && (Xbox.GetAButton())){
	        	elevatorHeight = SWITCH;
	        	limitTiming = 200;
	        }
	        if(limitTiming > 0)
	        	--limitTiming;
	        if(rumbleCount > 0)
	        	--rumbleCount;
	        frc::SmartDashboard::PutBoolean("Limit", limit);
	        frc::SmartDashboard::PutBoolean("Limit 1", tempLim1);
	        frc::SmartDashboard::PutBoolean("Limit 2", tempLim2);
	        frc::SmartDashboard::PutNumber("Height", elevatorHeight);
	        frc::Wait(0.005);
		}
	}
}
			void Test(){}

	//Reads the input from the ultrasonic rangefinder and converts it to inches.
	//Pre: Working Ultrasonic sensor
	//Post: none
	//Returns the inches from whatever the sensor is pointing at
	int UltraSensor (){
		return (DisStuff.GetVoltage()*40.297);
	}
	//Drives to distance
	//Pre: a valid distance
	//Post: drives forward the distance
	//Returns nothing
	void goForward(double distance) {
		bool drive(true);
		int error(0);
		EncoderL.Reset();
		EncoderR.Reset();
		const double target(ahrs->GetAngle());
		/* just go forward dammit
		while (drive && IsEnabled()) {
				frc::Wait(0.1);
				NavXForward(target);
		}*/

		while(drive && IsEnabled()) {
			frc::SmartDashboard::PutNumber("EncoderL",EncoderL.GetDistance());
			frc::SmartDashboard::PutNumber("EncoderR",EncoderR.GetDistance());
			if((-EncoderL.GetDistance() < (distance)) && (EncoderR.GetDistance() < (distance))) {
				NavXForward(target);
				frc::Wait(0.05);
				error = 0;
			}
			else if((-EncoderL.GetDistance() > (distance + 0.1)) && (EncoderR.GetDistance() > (distance + 0.1))) {
				NavXBackwards(target);
				frc::Wait(0.05);
				error = 0;
			}
			else {
				if(error < 100){
					myRobot.TankDrive(0, 0);
					error++;
					frc::Wait(0.005);
				}
				else {
					myRobot.TankDrive(0, 0);
					drive = false;
				}
			}

			myRobot.TankDrive(0,0);
		}
	}
	//Turns to an angle.
	//Pre: a valid angle
	//Post: drives forward the distance
	//Returns nothing
	void turnTo(double angle) {

		bool drive(true);
		double target = ahrs->GetAngle() + angle;
		int error(0);
		while(drive && IsEnabled()) {
			if(((ahrs->GetAngle()) < (target - 1))) {
				myRobot.TankDrive(0.6,-0.6);
				frc::Wait(0.05);
				error = 0;
			}
			else if(((ahrs->GetAngle()) > (target + 1))) {
				myRobot.TankDrive(-0.575,0.575);
				frc::Wait(0.05);
				error = 0;
			}
			else {
				if(error < 100){
					myRobot.TankDrive(0, 0);
					++error;
			}
				else{
				myRobot.TankDrive(0, 0);
					drive = false;
				}
			}
			myRobot.TankDrive(0,0);
		}
	}
	void NavXForward(const double target){
			static double fix;
			if(NavX)
				fix = (-(ahrs->GetAngle() - target)/100.0 + CALIBRATE_STRAIGHT);
			else
				fix = (-(EncoderL.Get() - EncoderR.Get())/100.0 + CALIBRATE_STRAIGHT);
			frc::SmartDashboard::PutNumber("Fix", fix);
			myRobot.TankDrive(fix, ROBOT_SPEED);
		}
	void NavXBackwards(const double target){
			static double fix;
			if(NavX)
				fix = (-(ahrs->GetAngle() - target)/100.0 - CALIBRATE_STRAIGHT);
			else
				fix = (-(EncoderL.Get() - EncoderR.Get())/100.0 - CALIBRATE_STRAIGHT);
			frc::SmartDashboard::PutNumber("Fix", fix);
			myRobot.TankDrive(-ROBOT_SPEED, fix);
		}
	void StopIntake(){
		Intake.Set(0);
	}
	void ToSwitch(){
		double time=0;
				if (ElevatorU.Get()) Elevator.Set(.8);
				while (ElevatorU.Get() && time<=2.5){
					frc::Wait(.1);
					time+=0.1;
				}
				Elevator.Set(0.0);
				OutPut();
	}
	void ToScale(){
		double time=0;
		if (ElevatorU.Get()) Elevator.Set(.5);
		while (ElevatorU.Get() && time<=4.0){
			frc::Wait(.1);
			time+=0.1;
		}
		Elevator.Set(0.0);
		OutPut();
	}
	void StartIntake(){
		Intake.Set(-0.5);
		//wait to tinker with
	}

	void OutPut(){
		Intake.Set(1.0);
		frc::Wait(1.0);
		Intake.Set(0.0);
	}
//FIELD CROSSING AUTOS NEED TO BE CHECKED THEY'RE ALL MOSTLY GUESSED
//add two feet to the end of each auto run until we fix the encoder
	void StartLeftSwitchLeftScaleLeft(){
		//left side auto (LL) two cube switch, scale
		goForward(12.0);//10.0
		turnTo(90.0);
		goForward(1.0);
		ToSwitch();
		turnTo(-90);
		goForward(7.0);//5.0
		turnTo(135);
		StartIntake();
		goForward(5.0);//3.0
		StopIntake();
		goForward(-5.0);//3.0
		turnTo(-135);
		goForward(7.5);//5.5
		ToScale();
}
	void StartLeftSwitchLeftScaleRight(){
		//left side auto (LR) double switch
		goForward(12.0);//10.0
		turnTo(90.0);
		goForward(1.0);
		ToSwitch();
		turnTo(-90);
		goForward(7.0);//5.0
		turnTo(135);
		StartIntake();
	    goForward(5.0);//3.0
		StopIntake();
		ToSwitch();
	}
	void StartLeftSwitchRightScaleLeft(){//CHECK THE ROUTES
		//left start auto (RL) switch
		goForward(17.5);//15.5
		turnTo(90);
		goForward(15.0);//17.0
		turnTo(90);
		//throw over other cubes
		ToSwitch();
	}
	void StartLeftSwitchRightScaleRight(){//CHECK TO ROUTES
		//left start auto (RR) switch
		goForward(17.5);//15.5
		turnTo(90);
		goForward(15.0);//17.0
		turnTo(90);
		//throw over other cubes
		ToSwitch();
	}
	void StartRightSwitchRightScaleLeft(){
	//right side auto (RL) double switch
		goForward(12.0);//10.0
		turnTo(-90.0);
		goForward(1.0);
		ToSwitch();
		turnTo(90);
		goForward(7.0);//5.0
		turnTo(-135);
		StartIntake();
	    goForward(5.0);//3.0
		StopIntake();
		ToSwitch();
	}
	void StartRightSwitchRightScaleRight(){
	//right side auto (RR) two cube switch, scale
		goForward(12.0);//10.0
		turnTo(-90.0);
		goForward(1.0);
		ToSwitch();
		turnTo(90);
		goForward(7.0);//5.0
		turnTo(-135);
		StartIntake();
		goForward(5.0);//3.0
		StopIntake();
		goForward(-5.0);//3.0
		turnTo(135);
		goForward(7.5);//5.5
		ToScale();
	}
	void StartRightSwitchLeftScaleLeft(){//CHECK THE ROUTES
		//right start auto (LL) switch
		goForward(17.5);//15.5
		turnTo(-90);
		goForward(15.0);//17.0
		turnTo(-90);
		//throw over other cubes
		ToSwitch();
	}
	void StartRightSwitchLeftScaleRight(){//CHECK THE ROUTES
		//right start auto (LR) switch
		goForward(17.5);//15.5
		turnTo(-90);
		goForward(15.0);//17.0
		turnTo(-90);
		//throw over other cubes
		ToSwitch();
		}

	void StartMiddleSwitchLeft(){
		goForward(4.0);// 3.0
		turnTo(-90);
		goForward(4.0);//3.0
		turnTo(90);
		goForward(4.0);
		ToSwitch();
		goForward(-3.0);
		turnTo(45);
		StartIntake();;
		goForward(2.0);
		StopIntake();
		goForward(-2.0);
		turnTo(-45);
		goForward(3.0);
		ToSwitch();
	}

	void StartMiddleSwitchRight(){
		goForward(4.0);// 3.0
		turnTo(90);
		goForward(4.0);//3.0
		turnTo(-90);
		goForward(4.0);
		ToSwitch();
		goForward(-3.0);
		turnTo(-45);
		StartIntake();;
		goForward(2.0);
		StopIntake();
		goForward(-2.0);
		turnTo(45);
		goForward(3.0);
		ToSwitch();
	}

	void PineappleTest(){
	//testing auto slot
	goForward(-5);//we need to test its ability to drive backwards in auto in order to get  multi cube autos
	}
};

START_ROBOT_CLASS(Robot)
