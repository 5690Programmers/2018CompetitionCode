#include <iostream>
#include <memory>
#include <string>
#include <XboxController.h>
#include <CameraServer.h>
#include <AHRS.h>
#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Drive/DifferentialDrive.h>
#include <Timer.h>
#include <PWMTalonSRX.h>
#include <SpeedControllerGroup.h>
#include <VictorSP.h>

const static double kP = 0.03f;
const static double kI = 0.00f;
const static double kD = 0.00f;
const static double kF = 0.00f;
const static double kToleranceDegrees = 2.0f;



class Robot: public frc::SampleRobot {
	//Controllers
	frc::XboxController Xbox{1};
	frc::Joystick Stick {0};

	//Add-ons
	frc::PIDController turnController;


	AHRS *ahrs
	double rotateToAngleRate;


	frc::VictorSP front_left {1};
	frc::VictorSP front_right {0};

	frc::SpeedControllerGroup left{front_left};
	frc::SpeedControllerGroup right{front_right};
	frc::DifferentialDrive myRobot {right, left};

	frc::SendableChooser<std::string> side;
	frc::SendableChooser<std::string> start;
	const std::string red = "Red";
	const std::string blue = "Blue";
	const std::string basic = "Go Forward";
	const std::string startright = "Right Start";
	const std::string startmid = "Middle Start";
	const std::string startleft = "Left Start";

public:

	virtual void PIDWrite(double output) {
	        this->rotateToAngleRate = output;

	Robot() {
		myRobot.SetExpiration(0.1);

		 rotateToAngleRate = 0.0f;
		 robotDrive.SetExpiration(0.1);
		 	 try {
		 		 ahrs = new AHRS(SPI::Port::kMXP);
		 	 } catch (std::exception& ex ) {
		        std::string err_string = "Error instantiating navX MXP:  ";
		        err_string += ex.what();
		        DriverStation::ReportError(err_string.c_str());
		     }
		        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		        turnController->SetInputRange(-180.0f,  180.0f);
		        turnController->SetOutputRange(-1.0, 1.0);
		        turnController->SetAbsoluteTolerance(kToleranceDegrees);
		        turnController->SetContinuous(true);
		    }


	}

	void RobotInit() {
		side.AddDefault(red, red);
		side.AddObject(blue, blue);
		side.AddObject(basic, basic);
		start.AddDefault(startright, startright);
		start.AddObject(startmid, startmid);
		start.AddObject(startleft, startleft);
		frc::SmartDashboard::PutData("Color Selected", &side);
		frc::SmartDashboard::PutData("Start Position Selected", &start);
		frc::CameraServer::GetInstance()->StartAutomaticCapture();

	}


	void Autonomous() {

		auto Start = start.GetSelected();
		auto Color = side.GetSelected();

		std::cout << "Auto selected:Start: " << Start << "Color: " << std::endl;

		if ((Start == startleft) && (Color == blue)) {
			//make auto for left start blue
			//Example to test AHRS.
			AHRS.Reset();
			myRobot.drive(0.5,0.5);
			while(IsAutonomous() && (AHRS.GetAngle() <= 90)){
				frc::SmartDashboard:PutNumber("Angle", AHRS.GetAngle());
				myRobot.Drive(-0.25, 0.8);
			}
			myRobot.Drive(0,0);

		} else if ((Start == startmid) && (Color == blue)) {
			//make auto for mid start blue

		} else if ((Start == startright) && (Color == blue)) {
			//make auto for right start blue

		} else if ((Start == startleft) && (Color == red)) {
			//make auto for left start red

		} else if ((Start == startmid) && (Color == red)) {
			//make auto for mid start red

		} else if ((Start == startright) && (Color == red)) {
			//make auto for right start red

		}else  {
			// Default Auto goes here. What do we want it to do no matter what?
			std::cout << "Running default Autonomous" << std::endl;
			myRobot.SetSafetyEnabled(false);
			myRobot.Drive(-0.25, 0.0);
			frc::Wait(8.0);
			myRobot.Drive(0.0, 0.0);
		}
	}


	void OperatorControl() override {
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {
			//myRobot.ArcadeDrive(-stick.GetY(), stick.GetX(), false); //Joy stick controls(right stick if xbox)
			if(Xbox.GetX(XboxController::JoystickHand(0)) > deadzone || Xbox.GetX(XboxController::JoystickHand(0)) < -deadzone)
				XboxX = Xbox.GetX(XboxController::JoystickHand(0));

			else
				XboxX = 0;

			if(Xbox.GetY(XboxController::JoystickHand(0)) > deadzone || Xbox.GetY(XboxController::JoystickHand(0)) < -deadzone)
				XboxY = Xbox.GetY(XboxController::JoystickHand(0));

			else
				XboxY = 0;

			myRobot.ArcadeDrive( XboxY, XboxX/1.25, true);


            bool reset_yaw_button_pressed = Xbox.GetStartButton();
            if ( reset_yaw_button_pressed ) {
                ahrs->ZeroYaw();
            }
            bool rotateToAngle = false;
            if (Xbox.GetYButton()) {
                turnController->SetSetpoint(0.0f);
                rotateToAngle = true;
            } else if (Xbox.GetBButton()) {
                turnController->SetSetpoint(90.0f);
                rotateToAngle = true;
            } else if (Xbox.GetAButton()) {
                turnController->SetSetpoint(179.9f);
                rotateToAngle = true;
            } else if (Xbox.GetXButton()) {
                turnController->SetSetpoint(-90.0f);
                rotateToAngle = true;
            }




            /*
			if (Xbox.GetTriggerAxis(XboxController::JoystickHand(0))) //0 is left trigger
			{
				Intake.Set(-0.5);
			} else {
				Intake.Set(0);
			}
			if (Xbox.GetTriggerAxis(XboxController::JoystickHand(1)))//1 is right trigger
			{
				Intake.Set(0.5);
			} else {
				Intake.Set(0)
			}
			/*
			//Move elevator arm up(X) and down(B)
			if(Xbox.GetXButton())
			{
				Arm.Set(.5);
			}
			else
			{
				Arm.Set(0);
			}
			if(stick.GetBButton())
			{
				Arm.Set(-0.5);
			}
			else
			{
				Arm.Set(0):
			}
			*/

			frc::Wait(0.005);
		}
	}

	void Test() override {

	}
};

START_ROBOT_CLASS(Robot);
