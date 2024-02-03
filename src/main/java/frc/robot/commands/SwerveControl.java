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

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.JoystickWrapper;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrive;

public class SwerveControl extends Command {
	JoystickWrapper driveJoystick;
	boolean driveStraight = false;
	double straightDegrees = 0;
	int delay=0;

	/**********************************************************************************
	 **********************************************************************************/

	public SwerveControl(SwerveDrive subsystem) {
		addRequirements(subsystem);
		driveJoystick = new JoystickWrapper(Robot.oi.driveController, 0.15);
	}

	/**********************************************************************************
	 **********************************************************************************/

	@Override
	public void initialize() {
		Robot.stopAutoCommand();
	}

	/**********************************************************************************
	 * Called every tick (20ms)
	 **********************************************************************************/

	@Override
	public void execute() {
		if (--delay < 0) { delay=0; }

		// X buttom aborts any running auto commands
		if (driveJoystick.isXButton()) {
			Robot.stopAutoCommand();
		}

		if (Robot.internalData.isAuto() || (Robot.isAutoCommand && Robot.autoMove == true)) {
			// Ignore user controls during Autonomous
			return;
		}

		// Get the driver inputs from the driver xbox controller
		double forwardBack = driveJoystick.getLeftStickY();
		double leftRight = driveJoystick.getLeftStickX();

		// Soften the rotate to 60%
		double rotate = driveJoystick.getRightStickX() * .6;

		// left Trigger enables slow mode
		if (driveJoystick.getLeftTrigger() > 0) {
			Robot.swerveDrive.driveSlow(true);
		} else {
			Robot.swerveDrive.driveSlow(false);
		}

		// Apply motor braking when the right trigger is pressed
		if (driveJoystick.getRightTrigger() > .5) {
			Robot.swerveDrive.brakesOn();
		} else {
			if (driveStraight == true) {
			    driveStraight = false;
				Robot.swerveDrive.brakesOff();
			}	
		}			

		// reset the gyro to zero fix any drift
		if (driveJoystick.isBButton()) {
			Robot.swerveDrive.resetYaw();
		}
		
		if (driveJoystick.isAButton()) {
			double dis = SmartDashboard.getNumber("Distance", 24);
			if (Robot.doAutoCommand()) {
				Robot.autoMove = true;
				Robot.autoCommand = new AutoDrive(.3, 0, 0, dis, 500);
				Robot.autoCommand.schedule();
			}
		}

    	if (driveJoystick.isStartButton()) {
			if (delay == 0) {
			    Robot.swerveDrive.toggleFullSpeed();
				delay=100;
			}	
		}	

		if (driveJoystick.isYButton()) {
			if (Robot.doAutoCommand()) {
				Robot.autoMove = true;
				Robot.autoCommand = new AutoTest();
				Robot.autoCommand.schedule();
			}
		}

		if (rotate == 0) {
			// if no rotate input specified, we are going to drive straight
			if (driveStraight != true) {
				// If driveStraight isn't set, save the current angle
				straightDegrees = Robot.navxMXP.getAngle();      
				driveStraight = true;
			} else {
				// clear drive straight since we are rotating
				driveStraight = false;
			}

		}


		SmartDashboard.putBoolean("A Pressed", driveJoystick.isAButton());

		Robot.swerveDrive.Drive(forwardBack, leftRight, rotate, driveStraight, straightDegrees);

	}
}

///////////////////////////////////////////////////////////////////////////////////////
// Controls
//
// Left Joystick - Driver Relative Robot Movement
//
// Right Joystick - Robot Rotation
//
// Left Trigger - Slow Mode
// Right Trigger - Brake Mode
//
// B Button - Reset Gyro to 0, do it when front of robot is facing directly away
// A Button -  Execute AutoDrive
//
/////////////////////////////////////////////////////////////////////////////////////// from
/////////////////////////////////////////////////////////////////////////////////////// the
/////////////////////////////////////////////////////////////////////////////////////// driver
//
