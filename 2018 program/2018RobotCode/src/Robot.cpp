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

class Robot: public frc::SampleRobot {

	frc::Spark Right1 {1};
	frc::Spark Right2 {2};
	frc::Spark Left1 {3};
	frc::Spark Left2 {4};
//the values of these need to be changed when it is actually wired
	frc::VictorSP IntakeL{5}; //see Intake controller group
	frc::VictorSP IntakeR{6}; //see Intake controller group
	frc::VictorSP Elevator{7}; //right and left triggers
	frc::VictorSP Folder{8};  //start and select
	frc::VictorSP Climber{0}; //right bumper

	frc::SpeedControllerGroup Intake{IntakeL, IntakeR}; //A, B, C, D buttons
	//driving speed controllers
	frc::SpeedControllerGroup GroupL{ Left1, Left2};
	frc::SpeedControllerGroup GroupR{ Right1, Right2};

	frc::DifferentialDrive myRobot {GroupL, GroupR};

	frc::XboxController Xbox { 0 };
//do the values for the encoders have to match those of the motors they are attached to?
	frc::Encoder EncoderR {0, 1, false, frc::Encoder::EncodingType::k4X};
	frc::Encoder EncoderL {2, 3, false, frc::Encoder::EncodingType::k4X};

	frc::AnalogInput DisStuff {3};

	frc::SendableChooser<std::string> chooser;

	const std::string autoNameMiddle = "StartMiddle";
	const std::string autoNameRight = "StartLeft";
	const std::string autoNameLeft = "StartRight";

	AHRS *ahrs;
	double rotateToAngleRate = 0.0;


public:
	//Gets the ultrasonic sensor's value
	int Ultrasensor();


	Robot()
	{

		//Testing

		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		myRobot.SetExpiration(0.1);


		EncoderL.SetMaxPeriod(0.1);
		EncoderL.SetMinRate(10);
		EncoderL.SetDistancePerPulse(0.003);  //.0104 //5 //.004
		EncoderL.SetReverseDirection(true);
		EncoderL.SetSamplesToAverage(7);

		EncoderR.SetMaxPeriod(0.1);
		EncoderR.SetMinRate(10);
		EncoderR.SetDistancePerPulse(0.003);//.0104
		EncoderR.SetReverseDirection(false);
		EncoderR.SetSamplesToAverage(7);

		ahrs = new AHRS(SPI::Port::kMXP);

	}

	void RobotInit() {
		 {

		        try {
		            //ahrs = new AHRS(SPI::Port::kMXP);
		            //ahrs = new AHRS(I2C::Port::kMXP);
		            ahrs->EnableLogging(true);
		        } catch (std::exception& ex ) {
		            std::string err_string = "Error instantiating navX MXP:  ";
		            err_string += ex.what();
		            DriverStation::ReportError(err_string.c_str());
		        }
			}

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
		if(autoSelected == autoNameLeft){
					//[]is start of start of scale and '' is start of switch
				if(gameData[0]=='L'){
					if(gameData[1]== 'L'){
						StartLeftSwitchLeftScaleLeft();
					}
					else{
						StartLeftSwitchLeftScaleRight();
					}
				}
				else if(gameData[0]=='R'){
					if(gameData[1]== 'L'){
						StartLeftSwitchRightScaleLeft();
					}
					else{
						StartLeftSwitchRightScaleRight();
					}
				}
		
		} //next section right side
		if(autoSelected == autoNameRight){

						if(gameData[0]=='L'){
							if(gameData[1]== 'L'){
								StartRightSwitchLeftScaleLeft();
							}
							else{
								StartRightSwitchLeftScaleRight();
							}
						}
						else if(gameData[0]=='R'){
							if(gameData[1]== 'L'){
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

		//double Correction = (-EncoderL.GetDistance()*0.0104) - (EncoderR.GetDistance()*0.0104);

		while (IsOperatorControl() && IsEnabled()) {

			SmartDashboard::PutNumber( "Left DisNotRaw: ", (EncoderL.GetDistance()*0.0104));//Left Motor Distance in inches  //*0.0104
			SmartDashboard::PutNumber( "Right DisNotRaw: ", (EncoderR.GetDistance()*0.0104));//Right Motor Distance in inches  //*0.0104
			SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());//Is the NavX connected
		    SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());//What angle the robot is at
			SmartDashboard::PutNumber(  "IMU_TotalYaw",         ahrs->GetAngle());//What angle the robot is at for real
			SmartDashboard::PutBoolean( "IMU_IsMoving",         ahrs->IsMoving());//Is the robot moving?
			SmartDashboard::PutNumber(  "Velocity_X",           ahrs->GetVelocityX() );//How fast robot is in X direction
			SmartDashboard::PutNumber(  "Velocity_Y",           ahrs->GetVelocityY() );//How fast robot is in Y direction
	        SmartDashboard::PutNumber(  "Displacement_X",       ahrs->GetDisplacementX() );//How far robot is in X direction
			SmartDashboard::PutNumber(  "Displacement_Y",       ahrs->GetDisplacementY() );//How far robot is in Y direction

//Tank with adjustments
			if(Xbox.GetX(XboxController::JoystickHand(0)) > deadzone || Xbox.GetX(XboxController::JoystickHand(0)) < -deadzone)
				XboxX = Xbox.GetX(XboxController::JoystickHand(0));

			else
				XboxX = 0;

			if(-Xbox.GetY(XboxController::JoystickHand(0)) > deadzone || -Xbox.GetY(XboxController::JoystickHand(0)) < -deadzone)
				XboxY = -Xbox.GetY(XboxController::JoystickHand(0));

			else
				XboxY = 0;

			//Button Commands}
			 if (Xbox.GetBackButton()) {
				 ahrs->ZeroYaw();
				 EncoderL.Reset();
				 EncoderR.Reset();
			 }
	        if(Xbox.GetBumper(XboxController::JoystickHand(1)))
				myRobot.TankDrive(.5, .5);

			/*else if(Xbox.GetBumper(XboxController::JoystickHand(0)))
				//turnTo(90.0);
				myRobot.TankDrive(-1,-1);*/

//unnote when needed^

			 else
				myRobot.ArcadeDrive( XboxY, XboxX/1.25, true);
	        //elevator
	        if (Xbox.GetTriggerAxis(XboxController::JoystickHand(1)) > 0){ //Limit switch stuff eventually
	        	Elevator.Set(Xbox.GetTriggerAxis(XboxController::JoystickHand(1)));
	        }else if (Xbox.GetTriggerAxis(XboxController::JoystickHand(0)) > 0){ //Limit switch stuff eventually
	        	Elevator.Set(-Xbox.GetTriggerAxis(XboxController::JoystickHand(0)));
	        }else{
	        	Elevator.Set(0);
	        }
	        //climber
	        if (Xbox.GetBumper(XboxController::JoystickHand(0))){
	        	Climber.Set(0.75);
	        }else {
	        	Climber.Set(0);
	        }
	        //folder
	        if (Xbox.GetStartButton()){ //Limit switch stuff eventually
	        	Folder.Set(0.25);
	        }else if (Xbox.GetBackButton()){ //Limit switch stuff eventually
	        	Folder.Set(0.25);
	        }else{
	        	Folder.Set(0);
	        }
	        //intake
	        if (Xbox.GetAButton()){
	        	Intake.Set(0.25);
	        }else if (Xbox.GetBButton()){
	        	Intake.Set(0.5);
	        }else if (Xbox.GetXButton()){
	        	Intake.Set(-0.25);
	        }else if (Xbox.GetYButton()){
	        	Intake.Set(-0.5);
	        }else{
	        	Intake.Set(0);
	        }

			 //Variable Updates
			//Correction = (-EncoderL.GetDistance()*0.0104) - (EncoderR.GetDistance()*0.0104);
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
		while(drive) {
			if((-EncoderL.GetDistance() < (distance - 2)) && (EncoderR.GetDistance() < (distance - 2))) {
				myRobot.TankDrive(0.5, 0.5);
				frc::Wait(0.05);
				error = 0;
			}
			else if((-EncoderL.GetDistance() > (distance + 2)) && (EncoderR.GetDistance() > (distance + 2))) {
				myRobot.TankDrive(-0.5, -0.5);
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
		int error(0);
		double target = ahrs->GetAngle() + angle;
		while(drive) {
			if(((ahrs->GetAngle()) < (target - 1))) {
				myRobot.TankDrive(0.5,-0.5);
				frc::Wait(0.05);
				error = 0;
			}
			else if(((ahrs->GetAngle()) < (target + 1))) {
				myRobot.TankDrive(-0.5,0.5);
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
	void StartLeftSwitchLeftScaleLeft(){
				//left side auto (LL)
		goForward(10.0);//10.0
		turnTo(90.0);
		goForward(3.0);
		//drop cube
		goForward(-1.0);
		turnTo(-90.0);
		goForward(5.0);
		turnTo(135.0);
		//start intake
		goForward(2.0);
		//stop intake
		goForward(-2.0);
		turnTo(-135.0);
		goForward(6.0);
		//place cube on scale m
}
	void StartLeftSwitchLeftScaleRight(){
		goForward(10.0);
		turnTo(90.0);
		goForward(3.0);
		//drop cube
		goForward(-1.0);
		turnTo(-90.0);
		goForward(5.0);
		turnTo(135.0);
		//start intake -
		goForward(2.0);
		//stop intake
		goForward(-2.0);
		turnTo(-45.0);
		goForward(14.0);
		turnTo(-90.0);
		goForward(6.0);
		//place cube on scale
	}
	void StartLeftSwitchRightScaleLeft(){

		goForward(15.5);
		turnTo(90);
		goForward(20.0);
		turnTo(90);
		goForward(5.5);
		turnTo(90.0);
		goForward(4.0);
		//drop cube
		goForward(-1.0);
		turnTo(90.0);
		goForward(5.5);
		turnTo(-90);
		goForward(15.0);
		turnTo(140);
		//start intake
		goForward(3.0);
		//stop intake
		goForward(-3.0);
		turnTo(-165);
		goForward(5.0);
		//place cube on scale
}
	void StartLeftSwitchRightScaleRight(){

		goForward(15.5);
		turnTo(90);
		goForward(20.0);
		turnTo(90);
		goForward(5.5);
		turnTo(90.0);
		goForward(4.0);
		//drop cube
		goForward(-1.0);
		turnTo(90.0);
		goForward(5.0);
		turnTo(-135.0);
		//start intake
		goForward(2.0);
		//stop intake
		goForward(-2.0);
		turnTo(135.0);
		goForward(6.0);
		//place cube on scale */
	}
	void StartRightSwitchLeftScaleLeft(){
		goForward(10.0);
		turnTo(-90.0);
		goForward(3.0);
		//drop cube
		goForward(-1.0);
		turnTo(90.0);
		goForward(5.0);
		turnTo(-135.0);
		//start intake
		goForward(2.0);
		//stop intake
		goForward(-2.0);
		turnTo(135.0);
		goForward(6.0);
		//place cube on scale


	}
	void StartRightSwitchLeftScaleRight(){
		goForward(10.0);
		turnTo(-90.0);
		goForward(3.0);
		//drop cube
		goForward(-1.0);
		turnTo(90.0);
		goForward(5.0);
		turnTo(-135.0);
		//start intake -
		goForward(2.0);
		//stop intake
		goForward(-2.0);
		turnTo(45.0);
		goForward(14.0);
		turnTo(90.0);
		goForward(6.0);
		//place cube on scale
	}
	void StartRightSwitchRightScaleLeft(){
		goForward(15.5);
		turnTo(-90);
		goForward(20.0);
		turnTo(-90);
		goForward(5.5);
		turnTo(-90.0);
		goForward(4.0);
		//drop cube
		goForward(-1.0);
		turnTo(-90.0);
		goForward(5.5);
		turnTo(90);
		goForward(15.0);
		turnTo(-140);
		//start intake
		goForward(3.0);
		//stop intake
		goForward(-3.0);
		turnTo(165);
		goForward(5.0);
		//place cube on scale
	}
	void StartRightSwitchRightScaleRight(){
		goForward(10.0);
	    turnTo(-90.0);
		goForward(4.0);
		//drop cube
        goForward(-1.0);
		turnTo(90.0);
		goForward(5.0);
		turnTo(-135.0);
		//start intake
		goForward(2.0);
		//stop intake
		goForward(-2.0);
		turnTo(135.0);
		goForward(6.0);
		//place cube on scale
	}

};

START_ROBOT_CLASS(Robot)
