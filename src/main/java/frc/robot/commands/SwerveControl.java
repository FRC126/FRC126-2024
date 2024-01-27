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

		Robot.swerveDrive.Drive(y1, x1, x2);

		if (driveJoystick.isBButton()) {
			Robot.swerveDrive.resetYaw();
		}
		
		SmartDashboard.putBoolean("A Pressed", driveJoystick.isAButton());

		if (driveJoystick.isAButton()) {
			double dis = SmartDashboard.getNumber("Distance", 24);
			if (Robot.doAutoCommand()) {
				Robot.swerveDrive.resetEncoders();
				Robot.autoMove = true;
				Robot.autoCommand = new AutoDrive(1, 0, 0, dis);
				Robot.autoCommand.schedule();
			}
		}

		if (driveJoystick.getRightTrigger() > 0) {
			Robot.swerveDrive.brakesOn();
		} else {
			Robot.swerveDrive.brakesOff();
		}

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
