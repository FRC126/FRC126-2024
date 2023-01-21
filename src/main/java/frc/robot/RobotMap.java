/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2022 Code       
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

    // Neo Motor Can Bus ID's
    public static int intakeMotor1CanID = 6;
	public static int intakeMotor2CanID = 7;
	public static int feederMotorCanID = 8;
	
	// Driver Motor Can ID's
	public static int leftDriveMotorCanID1 = 20;
	public static int leftDriveMotorCanID2 = 21;
	public static int rightDriveMotorCanID1 = 22;
	public static int rightDriveMotorCanID2 = 23;

	// Thrower Motor Can ID's
	public static int throwerMotorCanID1 = 9;
	public static int throwerMotorCanID2 = 10;

	// Climber Motor Can ID's
	public static int climberMotorLCanID = 11;
	public static int climberMotorRCanID = 12;
	
	//Motor Inversions
	public static int intakeMotor1Inversion;
	public static int intakeMotor2Inversion;
	public static int feederMotorInversion;

	public static int left1Inversion;
	public static int left2Inversion;
	public static int right1Inversion;
	public static int right2Inversion;
	public static int throwerMotor1Inversion;
	public static int throwerMotor2Inversion;
	public static int climberMotorLInversion;
	public static int climberMotorRInversion;

	//Position Calibrations
	public static void setRobot(double robotID){
		if(robotID == 0) { 
			// 2022 DriveBase
			intakeMotor1Inversion = 1; // Motor inversions
			intakeMotor2Inversion = 1;
			feederMotorInversion = -1;
			climberMotorLInversion = 1;
			climberMotorRInversion = -1;
			left1Inversion     = 1;
			left2Inversion     = 1;
			right1Inversion    = -1;
			right2Inversion    = -1;
			throwerMotor1Inversion = -1;
			throwerMotor2Inversion = 1;
		} else { 
			// 2022 Breadboard
			intakeMotor1Inversion = 1; // Motor inversions
			intakeMotor2Inversion = 1;
			feederMotorInversion = 1;
			climberMotorLInversion = 1;
			climberMotorRInversion = 1;
			left1Inversion     = 1;
			left2Inversion     = 1;
			right1Inversion    = 1;
			right2Inversion    = 1;
			throwerMotor1Inversion = -1;
			throwerMotor2Inversion = 1;
		}
	}

}
