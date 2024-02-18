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
	public static final int LidarChannel = 5; 

	///////////////////////////////////////////////////////////////////////////
	//Pickup Motor Can ID's
	public static final int PickupCanID = 50;

    ///////////////////////////////////////////////////////////////////////////
	//Pickup Motor Can ID's
	public static final int ClimberCanID = 51;

	///////////////////////////////////////////////////////////////////////////
	// Swerve Drive Motors 
    public static int swerveFrontRightDriveCanID;
    public static int swerveFrontRightTurnCanID;
    public static int swerveFrontLeftDriveCanID;
    public static int swerveFrontLeftTurnCanID;
    public static int swerveRearRightDriveCanID;
    public static int swerveRearRightTurnCanID;
    public static int swerveRearLeftDriveCanID;
    public static int swerveRearLeftTurnCanID;

	///////////////////////////////////////////////////////////////////////////
	// Swerve Drive Encoders
	public static int SwerveFrontRightEncoderCanID;
	public static int SwerveFrontLeftEncoderCanID;
	public static int SwerveRearRightEncoderCanID;
	public static int SwerveRearLeftEncoderCanID;

	/************************************************************************
	 * 
	 ************************************************************************/
	
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
			swerveFrontRightDriveCanID = 41;
			swerveFrontRightTurnCanID = 40;
			swerveFrontLeftDriveCanID = 42;
			swerveFrontLeftTurnCanID = 43;
			swerveRearRightDriveCanID = 47;
			swerveRearRightTurnCanID = 46;
			swerveRearLeftDriveCanID = 44;
			swerveRearLeftTurnCanID = 45;

			///////////////////////////////////////////////////////////////////////////
			// Swerve Drive Encoders
			SwerveFrontRightEncoderCanID = 30;
			SwerveFrontLeftEncoderCanID = 31;
			SwerveRearRightEncoderCanID = 32;
			SwerveRearLeftEncoderCanID = 33;
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

