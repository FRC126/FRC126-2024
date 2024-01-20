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
	// Controls for Xbox 360 / Xbox One
	public static int lStickX = 0; // Left stick X
	public static int lStickY = 1; // Left stick Y
	public static int rStickX = 4; // Right stick X
	public static int rStickY = 5; // Right stick Y
	public static int Rtrigger = 3; // Right trigger
	public static int Ltrigger = 2; // Left trigger
	public static int xboxA = 1; // A
	public static int xboxB = 2; // B
	public static int xboxX = 3; // X
	public static int xboxY = 4; // Y
	public static int xboxLTrig = 5; // Left trigger button
	public static int xboxRTrig = 6; // Right trigger button
	public static int xboxBack = 7; // Back
	public static int xboxStart = 8; // Start
	public static int xboxLStick = 9; // Left stick button
	public static int xboxRStick = 10; // Right stick button

	// Pneumatic Controller Can ID
	public static int PneumaticID = 2;
	
   	// Driver Motor Can ID's
	public static int leftDriveMotorCanID1 = 12;
	public static int leftDriveMotorCanID2 = 13;
	public static int rightDriveMotorCanID1 = 10;
	public static int rightDriveMotorCanID2 = 11;

  	// Driver Motor Can ID's
	public static int protoMotorOneCanID = 20;
	public static int protoMotorTwoCanID = 21;

	// Swerve Drive Motors 
    public static int swerveFrontRightDriveCanID = 40;
    public static int swerveFrontRightTurnCanID = 41;
    public static int swerveFrontLeftDriveCanID = 43;
    public static int swerveFrontLeftTurnCanID = 42;
    public static int swerveRearRightDriveCanID = 46;
    public static int swerveRearRightTurnCanID = 47;
    public static int swerveRearLeftDriveCanID = 44;
    public static int swerveRearLeftTurnCanID = 45;

	// Swerve Drive Encoders
	public static int SwerveFrontRightEncoderCanID = 30;
	public static int SwerveFrontLeftEncoderCanID = 31;
	public static int SwerveRearRightEncoderCanID = 33;
	public static int SwerveRearLeftEncoderCanID = 32;

	//Motor Inversions
    public static int left1Inversion;
	public static int left2Inversion;
	public static int right1Inversion;
	public static int right2Inversion;

	//Position Calibrations
	public static void setRobot(double robotID){
		if(robotID == 0) { 
			// 2024 DriveBase
			left1Inversion     = 1;
			left2Inversion     = 1;
			right1Inversion    = -1;
			right2Inversion    = -1;
		} else { 
			// 2024 Breadboard
			left1Inversion     = 1;
			left2Inversion     = 1;
			right1Inversion    = 1;
			right2Inversion    = 1;
		}
	}

/************************************************************************************************************
Controls
Driver:
Left Joystick Y-Axis: Forward and Backwards
Right Joysick X-Axis: Left and Right
POV Left: Shift robot left
POV Right: Shift robot right
Left Trigger: Hold down while driving for slow mode
Right Trigger: Hold down while driving to go straight, and apply motor braking when no throttle input.
 
AutoCommands:
X Button: Cancel any running auto routines
B Button: Turn 180 degrees
 
 Operator:

Auto Commands:
X Button: Cancel any running auto routines

BACK BUTTON: Holding the back button while using the manual controls will ignore encoders while 
moving components

*/
}
