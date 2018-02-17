/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Drive/DifferentialDrive.h>
#include <SampleRobot.h>
#include <Joystick.h>
#include <VictorSP.h>
#include <SpeedControllerGroup.h>
#include <WPILib.h>
#include <iostream>
#include <string>
#include <AHRS.h>
#include <Encoder.h>
#include <XboxController.h>
/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot : public frc::SampleRobot {
	frc::VictorSP left {3};
	frc::VictorSP left2 {2};
	//right side
	frc::VictorSP right {1};
	frc::VictorSP right2 {0};

	frc::SpeedControllerGroup GroupL{ left, left2};
	frc::SpeedControllerGroup GroupR{ right, right2};

	frc::DifferentialDrive myRobot {GroupR, GroupL};
	frc::XboxController Xbox { 0 };

	frc::SendableChooser<std::string> chooser;

	frc::Encoder EncoderR {0, 1, false, frc::Encoder::EncodingType::k4X};
	frc::Encoder EncoderL {2, 3, false, frc::Encoder::EncodingType::k4X};
	AHRS *ahrs;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";

public:
	Robot(){
		ahrs = new AHRS(SPI::Port::kMXP);
	}


	void RobotInit(){
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
	}
	void TeleopPeriodic() {
		double deadzone = 0.2;
			double XboxY;
			double XboxX;
		if(Xbox.GetX(XboxController::JoystickHand(0)) > deadzone || Xbox.GetX(XboxController::JoystickHand(0)) < -deadzone)
			XboxX = Xbox.GetX(XboxController::JoystickHand(0));

		else
			XboxX = 0;

		if(Xbox.GetY(XboxController::JoystickHand(0)) > deadzone || Xbox.GetY(XboxController::JoystickHand(0)) < -deadzone)
			XboxY = Xbox.GetY(XboxController::JoystickHand(0));

		else
			XboxY = 0;

		myRobot.ArcadeDrive( XboxY, -XboxX/1.25, true);

		if(Xbox.GetAButton())
			turnTo(90);

	}
	void Autonomous() {
//comment what auto does what and make sure the top chooser matches
		auto autoSelected = chooser.GetSelected();
		// std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;
        goForward(-9.0);
	}

	void Test(){

	}
	void goForward(double distance) {
		bool drive(true);
		int error(0);
		EncoderL.Reset();
		EncoderR.Reset();
		while(drive) {
			if(((-EncoderL.GetDistance()*0.0104) < (distance - 2)) && ((EncoderR.GetDistance()*0.0104) < (distance - 2))) {
				myRobot.TankDrive(0.5, 0.5);
				frc::Wait(0.05);
				error = 0;
			}
			else if(((-EncoderL.GetDistance()*0.0104) > (distance + 2)) && ((EncoderR.GetDistance()*0.0104) > (distance + 2))) {
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
					frc::SmartDashboard::PutNumber("Angle", ahrs->GetAngle());
					if(((ahrs->GetAngle()) < (target - 2))) {
						myRobot.TankDrive(-0.5, 0.5);
						frc::Wait(0.05);
						error = 0;
					}
					else if(((ahrs->GetAngle()) < (target + 2))) {
						myRobot.TankDrive(0.5, -0.5);
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
				}
			}
	};


START_ROBOT_CLASS(Robot)
