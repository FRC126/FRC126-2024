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
import frc.robot.JoystickWrapper;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrive;

public class SwerveControl extends Command {
	JoystickWrapper driveJoystick;
	boolean driveStraight = false;
	double straightDegrees = 0;

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
		// X buttom aborts any running auto commands
		if (driveJoystick.isXButton()) {
			Robot.stopAutoCommand();
		}

		if (Robot.internalData.isAuto() || (Robot.isAutoCommand && Robot.autoMove == true)) {
			// Ignore user controls during Autonomous
			return;
		}

		double y1 = driveJoystick.getLeftStickY();
		double x1 = driveJoystick.getLeftStickX();
		double x2 = driveJoystick.getRightStickX();

		if (driveJoystick.getLeftTrigger() > 0) {
			Robot.swerveDrive.driveSlow(true);
		} else {
			Robot.swerveDrive.driveSlow(false);
		}

		if (driveJoystick.isBButton()) {
			Robot.swerveDrive.resetYaw();
		}
		
		SmartDashboard.putBoolean("A Pressed", driveJoystick.isAButton());

		if (driveJoystick.isAButton()) {
			double distanceDesired = SmartDashboard.getNumber(Robot.DISTANCE_DESIRED, 24);
			if (Robot.doAutoCommand()) {
				Robot.autoMove = true;
				Robot.autoCommand = new AutoDrive(.3, 0, 0, distanceDesired, 500);
				Robot.autoCommand.schedule();
			}
		}

		if (driveJoystick.isYButton()) {
			if (Robot.doAutoCommand()) {
				Robot.autoMove = true;
				Robot.autoCommand = new AutoTest();
				Robot.autoCommand.schedule();
			}
		}

		// Apply motor braking when the right trigger is pressed
		if (driveJoystick.getRightTrigger() > .5) {
			Robot.swerveDrive.brakesOn();
			if (driveStraight != true) {
                // Get the current angle from the Navx 
				straightDegrees = Robot.navxMXP.getAngle();      
				driveStraight = true;
			}
		} else {
			if (driveStraight == true) {
			    driveStraight = false;
				Robot.swerveDrive.brakesOff();
			}	
		}			

		Robot.swerveDrive.Drive(y1, x1, x2, driveStraight, straightDegrees);

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
// Y Button - Execute AutoTest
// B Button - Reset Gyro to 0, do it when front of robot is facing directly away from the driver
// A Button - Execute AutoDrive
//
/////////////////////////////////////////////////////////////////////////////////////// 