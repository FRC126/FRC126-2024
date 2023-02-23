/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2023 Code       
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
	
    // Tower Arm Can Bus ID
	public static int TowerArmMotorID = 25;
	public static int TowerArmMotorInversion;

	// Extension Can Bus ID
	public static int ArmExtensionMotorID = 26;
	public static int ArmExtensionMotorInversion;

	// Grabber Can Bus ID
	public static int GrabberMotorID = 27;
	public static int GrabberMotorInversion;
	
	// Driver Motor Can ID's
	public static int leftDriveMotorCanID1 = 12;
	public static int leftDriveMotorCanID2 = 13;
	public static int rightDriveMotorCanID1 = 10;
	public static int rightDriveMotorCanID2 = 11;

	//Motor Inversions
    public static int left1Inversion;
	public static int left2Inversion;
	public static int right1Inversion;
	public static int right2Inversion;

	// Grabber Parameters
	public static double grabberClosedPos=0;
	public static double grabberOpenPos=340;
	public static double grabberConePos=30;
	public static double grabberCubePos=250;
 
	// Tower Arm
	public static double towerArmRetractedPos=-0;
	public static double towerArmPickupPos=15;
	public static double towerArmFloorPickupPos=30;
	public static double towerArmExtendedHighPos=160;
	public static double towerArmExtendedMaxPos=175;
	public static double towerArmExtendedMidPos=143;
	public static double towerArmExtendedLowPos=50;

	// Arm Extension
	public static double armRetractedPos=0;
	public static double armExtendedPlacePos=340;
	public static double armExtendedPickupPos=40;
	public static double armExtendedPickupFloorPos=90;
	public static double armExtendedPlaceLow=140;
	public static double armExtendedMaxPos=385;
	
	//Position Calibrations
	public static void setRobot(double robotID){
		if(robotID == 0) { 
			// 2023 DriveBase
			TowerArmMotorInversion = 1; // Motor inversions
			GrabberMotorInversion = 1; // Motor inversions
			ArmExtensionMotorInversion = 1; // Motor inversions
			left1Inversion     = 1;
			left2Inversion     = 1;
			right1Inversion    = -1;
			right2Inversion    = -1;
		} else { 
			// 2023 Breadboard
			TowerArmMotorInversion = 1; // Motor inversions
			GrabberMotorInversion = 1; // Motor inversions
			ArmExtensionMotorInversion = 1; // Motor inversions
			left1Inversion     = 1;
			left2Inversion     = 1;
			right1Inversion    = 1;
			right2Inversion    = 1;
		}
	}

	/************************************************************************************************************
	 * Controls
	 * 
	 * Driver:
	 * 		Left Joystick Y-Axis: Forward and Backwards
	 * 		Right Joysick X-Axis: Left and Right
	 * 		POV Left: Shift robot left
	 * 		POV Right: Shift robot right
	 * 		Left Trigger: Slow Mode
	 * 		Right Trigger: Drive straight, apply motor braking when stopped.
	 * 		A Button: Climb and Balance
	 * 		X Button: Cancel any running auto routines
	 * 		B Button: Use lime light to center on pole
	 * 
	 * Operator:
	 * 		Left Joystick Y-Axis: Up moves arm up, Down moves arm down
	 * 		Right Joystick Y-Axis: Up moves arm extension in, Down moves arm extension out
	 * 		Right Trigger: Close Gripper
	 * 		Left Trigger: Open Gripper
	 * 
	 * 		Auto Commands:
	 * 			X Button: Cancel any running auto routines
	 *      
	 * 			A Button: Place Cone Low Position
	 * 			B Button: Place Cone Mid Position
	 * 			Y Button: Place Cone High Position
	 * 
	 * 			POV UP + A Button: Place Cube Low Position
	 * 			POV UP + B Button: Place Cube Mid Position
	 * 			POV UP + Y Button: Place Cube High Position
	 * 
	 *          POV LEFT + A Button: Pickup Cone		
	 * 		    POV LEFT + B Button: Pickup Cube
	 * 
	 *          POV LEFT + Y Button: Catch Cone
	 *          POV DOWN + Y Button: Catch Cube
	 *          POV RIGHT + Y Button: Knock Over Cone
	 * 
	 * 			BACK BUTTON: Ignore encoders while move ARM, EXTENSION or GRIPPER, Only for Zero'ing the robot
	 * 			START BUTTON: Zero all encoders
	 * 			POV UP + START BUTTON: Zero arm encoder
	 * 			POV RIGHT + START BUTTON: Zero arm extension encoder
	 * 			POV DOWN + START BUTTON: Zero grabber encoder
	 */
}
