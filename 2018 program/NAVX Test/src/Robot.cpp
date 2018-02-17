#include "WPILib.h"
#include "AHRS.h"
#include "networktables/NetworkTableInstance.h"

using namespace nt;

/**
 * This is a demo program providing a real-time display of navX
 * MXP values.
 *
 * In the operatorControl() method, all data from the navX sensor is retrieved
 * and output to the SmartDashboard.
 *
 * The output data values include:
 *
 * - Yaw, Pitch and Roll angles
 * - Compass Heading and 9-Axis Fused Heading (requires Magnetometer calibration)
 * - Linear Acceleration Data
 * - Motion Indicators
 * - Estimated Velocity and Displacement
 * - Quaternion Data
 * - Raw Gyro, Accelerometer and Magnetometer Data
 *
 * As well, Board Information is also retrieved; this can be useful for debugging
 * connectivity issues after initial installation of the navX MXP sensor.
 *
 */
class Robot: public IterativeRobot
{
	std::shared_ptr<NetworkTable> table;
	Joystick stick; // only joystick
    AHRS *ahrs;
    int autoLoopCounter;

public:
    Robot() :
        table(NULL),
        stick(0),		// as they are declared above.
        ahrs(NULL),
        //lw(NULL),
        autoLoopCounter(0)
    {
    }

private:
    void RobotInit()
    {
    	NetworkTableInstance::GetDefault().GetTable("datatable");
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

    void AutonomousInit()
    {
        autoLoopCounter = 0;
    }

    void AutonomousPeriodic()
    {
        if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
        {
            autoLoopCounter++;
        }
    }

    void TeleopInit()
    {

    }

    void TeleopPeriodic()
    {
        if ( !ahrs ) return;

        bool reset_yaw_button_pressed = DriverStation::GetInstance().GetStickButton(0,1);
        if ( reset_yaw_button_pressed ) {
            ahrs->ZeroYaw();
        }
//all the notes are things im putting into the real code feel free to cut and add things if you guys think i should- Camden
        SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());
        SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());//this one
        SmartDashboard::PutNumber(  "IMU_Pitch",            ahrs->GetPitch());
        SmartDashboard::PutNumber(  "IMU_Roll",             ahrs->GetRoll());
        SmartDashboard::PutNumber(  "IMU_CompassHeading",   ahrs->GetCompassHeading());//prolly?
        SmartDashboard::PutNumber(  "IMU_Update_Count",     ahrs->GetUpdateCount());//maybe?
        SmartDashboard::PutNumber(  "IMU_Byte_Count",       ahrs->GetByteCount());
        SmartDashboard::PutNumber(  "IMU_Timestamp",        ahrs->GetLastSensorTimestamp());//maybe?

        /* These functions are compatible w/the WPI Gyro Class */
        SmartDashboard::PutNumber(  "IMU_TotalYaw",         ahrs->GetAngle());//this one
        SmartDashboard::PutNumber(  "IMU_YawRateDPS",       ahrs->GetRate());//this one

        SmartDashboard::PutNumber(  "IMU_Accel_X",          ahrs->GetWorldLinearAccelX());//maybe?
        SmartDashboard::PutNumber(  "IMU_Accel_Y",          ahrs->GetWorldLinearAccelY());//maybe?
        SmartDashboard::PutBoolean( "IMU_IsMoving",         ahrs->IsMoving());//maybe?
        SmartDashboard::PutNumber(  "IMU_Temp_C",           ahrs->GetTempC());
        SmartDashboard::PutBoolean( "IMU_IsCalibrating",    ahrs->IsCalibrating());//maybe?

        SmartDashboard::PutNumber(  "Velocity_X",           ahrs->GetVelocityX() );//this one
        SmartDashboard::PutNumber(  "Velocity_Y",           ahrs->GetVelocityY() );//this one
        SmartDashboard::PutNumber(  "Displacement_X",       ahrs->GetDisplacementX() );//this one
        SmartDashboard::PutNumber(  "Displacement_Y",       ahrs->GetDisplacementY() );//this one

        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
        /* NOTE:  These values are not normally necessary, but are made available   */
        /* for advanced users.  Before using this data, please consider whether     */
        /* the processed data (see above) will suit your needs.                     */

        SmartDashboard::PutNumber(  "RawGyro_X",            ahrs->GetRawGyroX());
        SmartDashboard::PutNumber(  "RawGyro_Y",            ahrs->GetRawGyroY());
        SmartDashboard::PutNumber(  "RawGyro_Z",            ahrs->GetRawGyroZ());
        SmartDashboard::PutNumber(  "RawAccel_X",           ahrs->GetRawAccelX());
        SmartDashboard::PutNumber(  "RawAccel_Y",           ahrs->GetRawAccelY());
        SmartDashboard::PutNumber(  "RawAccel_Z",           ahrs->GetRawAccelZ());
        SmartDashboard::PutNumber(  "RawMag_X",             ahrs->GetRawMagX());
        SmartDashboard::PutNumber(  "RawMag_Y",             ahrs->GetRawMagY());
        SmartDashboard::PutNumber(  "RawMag_Z",             ahrs->GetRawMagZ());
        SmartDashboard::PutNumber(  "IMU_Temp_C",           ahrs->GetTempC());
        /* Omnimount Yaw Axis Information                                           */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
        AHRS::BoardYawAxis yaw_axis = ahrs->GetBoardYawAxis();
        SmartDashboard::PutString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard::PutNumber(  "YawAxis",              yaw_axis.board_axis );

        /* Sensor Board Information                                                 */
        SmartDashboard::PutString(  "FirmwareVersion",      ahrs->GetFirmwareVersion());

        /* Quaternion Data                                                          */
        /* Quaternions are fascinating, and are the most compact representation of  */
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
        /* from the Quaternions.  If interested in motion processing, knowledge of  */
        /* Quaternions is highly recommended.                                       */
        SmartDashboard::PutNumber(  "QuaternionW",          ahrs->GetQuaternionW());
        SmartDashboard::PutNumber(  "QuaternionX",          ahrs->GetQuaternionX());
        SmartDashboard::PutNumber(  "QuaternionY",          ahrs->GetQuaternionY());
        SmartDashboard::PutNumber(  "QuaternionZ",          ahrs->GetQuaternionZ());

    }

    void TestPeriodic()
    {
        //lw->Run();
    }
};

START_ROBOT_CLASS(Robot);
