/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <Drive/DifferentialDrive.h>
#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Spark.h>
#include <Timer.h>
#include <PWMTalonSRX.h>
#include <SpeedControllerGroup.h>
/**
 * This is a demo program showing the use of the DifferentialDrive class.
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
class Robot : public frc::SampleRobot {
public:
	//fix according to the inputs
		frc::PWMTalonSRX top_left {0};
		frc::PWMTalonSRX front_left {1};
		frc::PWMTalonSRX back_left {2};
		frc::PWMTalonSRX top_right {3};
		frc::PWMTalonSRX front_right {4};
		frc::PWMTalonSRX back_right {5};

		frc::SpeedControllerGroup left{top_left, front_left, back_left};
		frc::SpeedControllerGroup right{top_right, front_right, back_right};

		frc::DifferentialDrive myRobot {right, left};
		frc::Joystick stick { 0 };


	Robot() {
		// Note SmartDashboard is not initialized here, wait until
		// RobotInit to make SmartDashboard calls
		m_robotDrive.SetExpiration(0.1);
	}

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select between different autonomous modes using the dashboard. The
	 * sendable chooser code works with the Java SmartDashboard. If you
	 * prefer the LabVIEW Dashboard, remove all of the chooser code and
	 * uncomment the GetString line to get the auto name from the text box
	 * below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void Autonomous() {
		std::string autoSelected = m_chooser.GetSelected();
		// std::string autoSelected = frc::SmartDashboard::GetString(
		// "Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		// MotorSafety improves safety when motors are updated in loops
		// but is disabled here because motor updates are not looped in
		// this autonomous mode.
		m_robotDrive.SetSafetyEnabled(false);

		if (autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
			std::cout << "Running custom Autonomous" << std::endl;
//testing out an auto with the new coding changes(really just heading changes)
			m_robotDrive.ArcadeDrive(0.5, 0.0, false);
			frc::Wait(1.0);
			m_robotDrive.CurvatureDrive(-0.5, 1.0, false);
			frc::Wait(.5);
			m_robotDrive.ArcadeDrive(0.5, 0.0, false);
			frc::Wait(1.0);
			m_robotDrive.ArcadeDrive(0.0, 0.0);//stop robot
			frc::Wait(2.0);
		} else {
			// Default Auto goes here
			std::cout << "Running default Autonomous" << std::endl;

			// Drive forwards at half speed for two seconds
			m_robotDrive.ArcadeDrive(-0.5, 0.0);
			frc::Wait(2.0);

			// Stop robot
			m_robotDrive.ArcadeDrive(0.0, 0.0);
		}
	}

	/*
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl() override {
		m_robotDrive.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {
			// Drive with arcade style (use right stick)
			m_robotDrive.ArcadeDrive(
					-m_stick.GetY(), m_stick.GetX());

			// The motors will be updated every 5ms
			frc::Wait(0.005);
		}
	}

	/*
	 * Runs during test mode
	 */
	void Test() override {}

private:
	// Robot drive system
	frc::Spark m_leftMotor{0};
	frc::Spark m_rightMotor{1};
	frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};

	frc::Joystick m_stick{0};

	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
};

START_ROBOT_CLASS(Robot)
