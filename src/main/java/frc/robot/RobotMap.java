/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2024 Code       
	Go get em gaels!

***********************************/

package frc.robot;

public class RobotMap {
    public static int NeoTicksPerRotation=42;
	public static double ThrowerGearRatio=750;

	///////////////////////////////////////////////////////////////////////////
	// Controls for Xbox 360 / Xbox One
	public static final int lStickX = 0; // Left stick X
	public static final int lStickY = 1; // Left stick Y
	public static final int rStickX = 4; // Right stick X
	public static final int rStickY = 5; // Right stick Y
	public static final int Rtrigger = 3; // Right trigger
	public static final int Ltrigger = 2; // Left trigger
	public static final int xboxA = 1; // A
	public static final int xboxB = 2; // B
	public static final int xboxX = 3; // X
	public static final int xboxY = 4; // Y
	public static final int xboxLTrig = 5; // Left trigger button
	public static final int xboxRTrig = 6; // Right trigger button
	public static final int xboxBack = 7; // Back
	public static final int xboxStart = 8; // Start
	public static final int xboxLStick = 9; // Left stick button
	public static final int xboxRStick = 10; // Right stick button

	///////////////////////////////////////////////////////////////////////////
 	// Thrower Motor Can ID's
	public static final int throwerTalonMotorOneCanID = 26;
	public static final int throwerTalonMotorTwoCanID = 27;
	public static final int throwerTriggerMotorCanID = 28;
	public static final int throwerClimberMotorLeftCanID = 24;
	public static final int throwerClimberMotorRightCanID = 25;

	///////////////////////////////////////////////////////////////////////////
	// Swerve Drive Motors 
    public static int swerveFrontRightDriveCanID = 40;
    public static int swerveFrontRightTurnCanID = 41;
    public static int swerveFrontLeftDriveCanID = 43;
    public static int swerveFrontLeftTurnCanID = 42;
    public static int swerveRearRightDriveCanID = 46;
    public static int swerveRearRightTurnCanID = 47;
    public static int swerveRearLeftDriveCanID = 44;
    public static int swerveRearLeftTurnCanID = 45;

	///////////////////////////////////////////////////////////////////////////
	// Swerve Drive Encoders
	public static int SwerveFrontRightEncoderCanID = 30;
	public static int SwerveFrontLeftEncoderCanID = 31;
	public static int SwerveRearRightEncoderCanID = 33;
	public static int SwerveRearLeftEncoderCanID = 32;

	///////////////////////////////////////////////////////////////////////////
	public static final int LidarChannel = 5; 

	///////////////////////////////////////////////////////////////////////////
	//Pickup Motor Can ID's
	public static final int PickupCanID = 50;

	//Position Calibrations
	public static void setRobot(double robotID){
		if(robotID == 0) { 
			// 2024 DriveBase

			///////////////////////////////////////////////////////////////////////////
			// Swerve Drive Motors 
			swerveFrontRightDriveCanID = 40;
			swerveFrontRightTurnCanID = 41;
			swerveFrontLeftDriveCanID = 43;
			swerveFrontLeftTurnCanID = 42;
			swerveRearRightDriveCanID = 46;
			swerveRearRightTurnCanID = 47;
			swerveRearLeftDriveCanID = 44;
			swerveRearLeftTurnCanID = 45;

			///////////////////////////////////////////////////////////////////////////
			// Swerve Drive Encoders
			SwerveFrontRightEncoderCanID = 30;
			SwerveFrontLeftEncoderCanID = 31;
			SwerveRearRightEncoderCanID = 33;
			SwerveRearLeftEncoderCanID = 32;
		} else { 
			// 2024 Official Robot

			///////////////////////////////////////////////////////////////////////////
			// Swerve Drive Motors 
			swerveFrontRightDriveCanID = 40;
			swerveFrontRightTurnCanID = 41;
			swerveFrontLeftDriveCanID = 43;
			swerveFrontLeftTurnCanID = 42;
			swerveRearRightDriveCanID = 46;
			swerveRearRightTurnCanID = 47;
			swerveRearLeftDriveCanID = 44;
			swerveRearLeftTurnCanID = 45;

			///////////////////////////////////////////////////////////////////////////
			// Swerve Drive Encoders
			SwerveFrontRightEncoderCanID = 30;
			SwerveFrontLeftEncoderCanID = 31;
			SwerveRearRightEncoderCanID = 33;
			SwerveRearLeftEncoderCanID = 32;
		}
	}
}


/******************************************************************************
Controls

Driver:
X Button: Cancel any running auto routines
 
Operator:

Auto Commands:
X Button: Cancel any running auto routines
******************************************************************************/

