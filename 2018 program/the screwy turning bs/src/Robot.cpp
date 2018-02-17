#include "WPILib.h"
#include "AHRS.h"


const static double kP = 0.05f;
const static double kI = 0.00f;
const static double kD = 0.00f;
const static double kF = 0.00f;


const static double kToleranceDegrees = 2.0f;



class Robot: public SampleRobot, public PIDOutput
{


    // Channels for the wheels
    const static int frontLeftChannel	= 2;
    const static int rearLeftChannel	= 3;
    const static int frontRightChannel	= 1;
    const static int rearRightChannel	= 0;

    const static int joystickChannel	= 0;

    RobotDrive robotDrive;	            // Robot drive system
    Joystick stick;			            // Driver Joystick
    AHRS *ahrs;                         // navX-MXP
    PIDController *turnController;      // PID Controller
    double rotateToAngleRate;           // Current rotation rate

public:
	Robot() :
            robotDrive(frontLeftChannel, rearLeftChannel,
                       frontRightChannel, rearRightChannel), // initialize variables in same
            stick(joystickChannel)                           // order as declared above.
    {
	    rotateToAngleRate = 0.0f;
      //  robotDrive.SetExpiration(0.1);
      // robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);	// invert left side motors
      //robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);	// change to match your robot
	    ahrs = new AHRS(SPI::Port::kMXP);
	    try {

            ahrs = new AHRS(SPI::Port::kMXP);
        } catch (std::exception ex ) {
            std::string err_string = "Error instantiating navX-MXP:  ";
            err_string += ex.what();
            DriverStation::ReportError(err_string.c_str());
        }
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController->SetInputRange(-180.0f,  180.0f);
        turnController->SetOutputRange(-1.0, 1.0);
        turnController->SetAbsoluteTolerance(kToleranceDegrees);
        turnController->SetContinuous(true);


     //   LiveWindow::GetInstance()->AddActuator("DriveSystem", "RotateController", turnController);
        if ( ahrs ) {
           // LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
        }
    }

    void OperatorControl()
    {
        robotDrive.SetSafetyEnabled(false);
        double turning;
        while (IsOperatorControl() && IsEnabled())
        {
            bool reset_yaw_button_pressed = stick.GetRawButton(1);
            if ( reset_yaw_button_pressed ) {
                ahrs->ZeroYaw();
            }
            bool rotateToAngle = false;
            if ( stick.GetRawButton(2)) {
                turnController->SetSetpoint(0.0f);
                rotateToAngle = true;
            } else if ( stick.GetRawButton(3)) {
                turnController->SetSetpoint(90.0f);
                rotateToAngle = true;
            } else if ( stick.GetRawButton(4)) {
                turnController->SetSetpoint(179.9f);
                rotateToAngle = true;
            } else if ( stick.GetRawButton(5)) {
                turnController->SetSetpoint(-90.0f);
                rotateToAngle = true;
            }
            double currentRotationRate;
            if ( rotateToAngle ) {
                turnController->Enable();
                currentRotationRate = rotateToAngleRate;
            } else {
                turnController->Enable();
                currentRotationRate = stick.GetTwist();
            }
          /*  try {

               // robotDrive.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(),
                                                 // currentRotationRate ,ahrs->GetAngle());
            } catch (std::exception ex ) {
                std::string err_string = "Error communicating with Drive System:  ";
                err_string += ex.what();
                DriverStation::ReportError(err_string.c_str());
            }
      */
            SmartDashboard::PutNumber("Setpoint", turnController->GetSetpoint());
        	SmartDashboard::PutBoolean("Keffefa", turnController->IsEnabled());
        	SmartDashboard::PutNumber("Mystery", rotateToAngleRate);
            Wait(0.005); // wait 5ms to avoid hogging CPU cycles
        }
    }
    void PIDWrite(double output) {
        rotateToAngleRate = output;
    }
};

START_ROBOT_CLASS(Robot);
