#include <iostream>
#include <memory>
#include <string>

#include <AHRS.h>

#include <Encoder.h>

#include <Joystick.h>
#include <XboxController.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Drive/DifferentialDrive.h>
#include <Timer.h>
#include <VictorSP.h>
#include <SpeedControllerGroup.h>

#include <WPILib.h>

#include "networktables/NetworkTableInstance.h"

/**
 * This is a demo program+++++-+ showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will
 * automatically call your Autonomous and OperatorControl methods at the right
 * time as controlled by the switches on the driver station or the field
 * controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
class Robot: public frc::SampleRobot {

	//left side
	frc::VictorSP left {3};
	frc::VictorSP left2 {2};
	//right side
	frc::VictorSP right {1};
	frc::VictorSP right2 {0};

	//climber
	frc::VictorSP Climber {4};
	//Intake
	frc::VictorSP Intake { 5 };
	frc::DoubleSolenoid IntakeArm { 0, 1 };

	frc::SpeedControllerGroup GroupL{ left, left2};
	frc::SpeedControllerGroup GroupR{ right, right2};

	frc::DifferentialDrive myRobot {GroupR, GroupL};
	frc::XboxController Xbox { 0 };
	frc::SendableChooser<std::string> chooser;



	frc::AnalogInput DisStuff {3};

	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";

	std::shared_ptr<NetworkTable> table;//vison tracking
	AHRS *ahrs;
	int autoLoopCounter;



public:// idk how to fix this
	Robot() :
        table(NULL),
        ahrs(NULL),
        //lw(NULL),
        autoLoopCounter(0)
	{

		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		myRobot.SetExpiration(0.1);
		//table = NetworkTable::GetTable("GRIP/Logan");//vision tracing

		ahrs = new AHRS(SPI::Port::kMXP);

	}

	void RobotInit() {
		 {

			 ahrs->Reset();

			 try {
					/***********************************************************************
					 * navX-MXP:
					 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
					 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
					 *
					 * navX-Micro:
					 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
					 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
					 *
					 * Multiple navX-model devices on a single robot are supported.
					 ************************************************************************/
		            //ahrs = new AHRS(SPI::Port::kMXP);
		            ahrs = new AHRS(I2C::Port::kMXP);
		            ahrs->EnableLogging(true);
		        } catch (std::exception& ex ) {
		            std::string err_string = "Error instantiating navX MXP:  ";
		            err_string += ex.what();
		            DriverStation::ReportError(err_string.c_str());
		        }
			}

		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
	}


	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void Autonomous() {
	 {
		 autoLoopCounter = 0;
	 }

		auto autoSelected = chooser.GetSelected();
		// std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;
		std::string gameData;
				gameData=frc::DriverStation::GetInstance().GetGameSpecificMessage();
				if(gameData[0]=='L')
				{

					//put code for switch on left
				}else{
					//Put code for the switch on the right here
				}
		if (autoSelected == autoNameCustom)
		{
			// Custom Auto goes here
			std::cout << "Running custom Autonomous" << std::endl;
			myRobot.SetSafetyEnabled(false);
			myRobot.CurvatureDrive(0.5, 0.0, false); //go straight for 1 second
			frc::Wait(1.0);                // for 2 seconds
			myRobot.CurvatureDrive(0.5,-1, true);//turn for 0.5 seconds
			frc::Wait(0.5);
			myRobot.CurvatureDrive(0.5, 0.0, false); //go straight for 1 second
			frc::Wait(1.0);
			myRobot.CurvatureDrive(0.0, 0.0, false);  // stop robot
			frc::Wait(1.0);

		} else {
			// Default Auto goes here
			std::cout << "Running default Autonomous" << std::endl;
			myRobot.SetSafetyEnabled(false);
			myRobot.CurvatureDrive(-0.5, 0.0, false); // drive forwards half speed
			frc::Wait(2.0);                // for 2 seconds
			myRobot.CurvatureDrive(0.0, 0.0, false);  // stop robot
		}
	}

	void AutonomousPeriodic()
	    {
	        if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
	        {
	            autoLoopCounter++;
	        }
	    }

	int UltraSensor (){
			return DisStuff.GetVoltage()*1000*(1/0.977)*(1/25.4);
		}
	/*
	 * Runs the motors with arcade steering.
	 *
	 */
	void OperatorControl() override {
	 {


		        if ( !ahrs ) return;

		        bool reset_yaw_button_pressed = DriverStation::GetInstance().GetStickButton(0,1);
		        if ( reset_yaw_button_pressed ) {
		            ahrs->ZeroYaw();
		        }

		myRobot.SetSafetyEnabled(true);
		double deadzone = 0.2;
		double XboxY;
		double XboxX;
		while (IsOperatorControl() && IsEnabled()) {

			/* SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());
			 SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());
			 SmartDashboard::PutNumber(  "IMU_CompassHeading",   ahrs->GetCompassHeading());
			 SmartDashboard::PutNumber(  "IMU_Update_Count",     ahrs->GetUpdateCount());
			 SmartDashboard::PutNumber(  "IMU_Timestamp",        ahrs->GetLastSensorTimestamp());

					        // These functions are compatible w/the WPI Gyro Class
			 SmartDashboard::PutNumber(  "IMU_TotalYaw",         ahrs->GetAngle());
			 SmartDashboard::PutNumber(  "IMU_YawRateDPS",       ahrs->GetRate());
			 SmartDashboard::PutNumber(  "IMU_Accel_X",          ahrs->GetWorldLinearAccelX());
			 SmartDashboard::PutNumber(  "IMU_Accel_Y",          ahrs->GetWorldLinearAccelY());
			 SmartDashboard::PutBoolean( "IMU_IsMoving",         ahrs->IsMoving());
			 SmartDashboard::PutBoolean( "IMU_IsCalibrating",    ahrs->IsCalibrating());

			 SmartDashboard::PutNumber(  "Velocity_X",           ahrs->GetVelocityX() );
			 SmartDashboard::PutNumber(  "Velocity_Y",           ahrs->GetVelocityY() );
			 SmartDashboard::PutNumber(  "Displacement_X",       ahrs->GetDisplacementX() );
			 SmartDashboard::PutNumber(  "Displacement_Y",       ahrs->GetDisplacementY() );

			 AHRS::BoardYawAxis yaw_axis = ahrs->GetBoardYawAxis();
			 SmartDashboard::PutString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
			 SmartDashboard::PutNumber(  "YawAxis",              yaw_axis.board_axis );

					        // Sensor Board Information
			 SmartDashboard::PutString(  "FirmwareVersion",      ahrs->GetFirmwareVersion());
			 SmartDashboard::PutNumber(  "QuaternionW",          ahrs->GetQuaternionW());
			 SmartDashboard::PutNumber(  "QuaternionX",          ahrs->GetQuaternionX());
			 SmartDashboard::PutNumber(  "QuaternionY",          ahrs->GetQuaternionY());
			 SmartDashboard::PutNumber(  "QuaternionZ",          ahrs->GetQuaternionZ());
*/
			//Tank with adjustments
			if(Xbox.GetX(XboxController::JoystickHand(0)) > deadzone || Xbox.GetX(XboxController::JoystickHand(0)) < -deadzone)
				XboxX = Xbox.GetX(XboxController::JoystickHand(0));

			else
				XboxX = 0;

			if(Xbox.GetY(XboxController::JoystickHand(0)) > deadzone || Xbox.GetY(XboxController::JoystickHand(0)) < -deadzone)
				XboxY = Xbox.GetY(XboxController::JoystickHand(0));

			else
				XboxY = 0;

			myRobot.ArcadeDrive( XboxY, -XboxX/1.25, true);


			//Climber Trigger
			if (Xbox.GetTriggerAxis(XboxController::JoystickHand(1)))
			{
				Climber.Set(-Xbox.GetTriggerAxis(XboxController::JoystickHand(1)));
			}else
			{
				Climber.Set(0);
			}


			//Intake
			if (Xbox.GetYButton())
				Intake.Set(0.5);

			else if(Xbox.GetAButton())
				Intake.Set(-0.25);

			else
				Intake.Set(0);

			//Placing
			if (Xbox.GetBButton()){
				myRobot.ArcadeDrive(-.25,0, false);
				Intake.Set(-0.35);
				IntakeArm.Set(DoubleSolenoid::Value(2));
				Wait(0.005);
				myRobot.ArcadeDrive(0,0, false);
				Intake.Set(0);
			}
			//intake arm
			if (Xbox.GetXButton() && IntakeArm.Get()!=1)
					{
				IntakeArm.Set(DoubleSolenoid::Value(1));


						Wait(0.5);
					}
					else if (Xbox.GetXButton() && IntakeArm.Get() != 2)
					{
						IntakeArm.Set(DoubleSolenoid::Value(2));
						Wait(0.5);
					}
			// wait for a motor update time
			frc::Wait(0.005);
		}
	}
	}

	/*
	 * Runs during test mode
	 */
	void Test() override {
	/* {
				while (IsEnabled()){
					std::vector<double> x = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
					std::vector<double> y = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
					std::vector<double> area = table->GetNumberArray("area", llvm::ArrayRef<double>());
					double bigest = 0;
					double num = 0;
					unsigned int Magic = 0;
					if ((area.size() > Magic) && (Xbox.GetRawButton(7) && Xbox.GetRawButton(8) && (UltraSensor() > 24))){
						for (unsigned int i = 0; i < area.size(); i++){
							if (bigest < area[i]) {
								bigest = area[i];
								num = i;
							}
						}
						if (area[num] > 600){
							myRobot.ArcadeDrive(0,0,false);
						}else if (area[num] < 600){
							myRobot.ArcadeDrive(0.3, -((x[num]-80)/230),false);
						}else {
							myRobot.ArcadeDrive(0,0,false);
						}
					}else{
						myRobot.ArcadeDrive(0,0,false);
						Wait(0.5);
					}
				}
		    }*/
	}
};

START_ROBOT_CLASS(Robot)
