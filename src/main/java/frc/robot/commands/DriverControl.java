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

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.*;	
import frc.robot.JoystickWrapper;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.CANSparkMax;

/**********************************************************************************
 **********************************************************************************/

public class DriverControl extends CommandBase {
	JoystickWrapper driveJoystick;
	
	/**********************************************************************************
	 **********************************************************************************/
	
    public DriverControl(WestCoastDrive subsystem) {
		addRequirements(subsystem);
		driveJoystick = new JoystickWrapper(Robot.oi.driveController, 0.15);
    }

	/**********************************************************************************
	 **********************************************************************************/
	
	@Override
	public void initialize() {
	}    

	/**********************************************************************************
	 * Called every tick (20ms)
	 **********************************************************************************/
	
	@Override
	public void execute() {
		if (Robot.internalData.isAuto()) {
			// Ignore user controls during Autonomous
			return;
		}

		// Get stick inputs
        double FB = driveJoystick.getLeftStickY();
        double LR = driveJoystick.getRightStickX();

		// Slow down the driver controls when the left trigger is pressed
		if (driveJoystick.getLeftTrigger() > .6) {
			LR=LR*.7;
			FB=FB*.3;
		}

		// Apply motor braking when the right trigger is pressed
		if (driveJoystick.getRightTrigger() > .6) {
			Robot.driveBase.brakesOn();
		} else {
			Robot.driveBase.brakesOff();
		}	

		// Turn off the brakes if operator is using the joysticks	  
		if ( FB > .1 || FB < -0.1 || LR > .1 || LR < -0.1 ) {
			Robot.driveBase.brakesOff();
		}

		/*
		// Cancel running auto's
		if (driveJoystick.isXButton()) {
			Robot.driveBase.stopAutoBalance();
			Robot.driveBase.stopAutoClimbBalance();
			Robot.driveBase.stopAutoMoveLeft();
			Robot.driveBase.stopAutoMoveRight();
		}

		// Auto balance the robot
		if (driveJoystick.isAButton()) {
			Robot.driveBase.doAutoBalance();
		}

		// Climb then auto balance the robot
		if (driveJoystick.isBButton()) {
			Robot.driveBase.doAutoClimbBalance();
		}

	    // Shift the Robot Left
		if (driveJoystick.getPovLeft()) {
			Robot.driveBase.doAutoMoveLeft();
		} else {
			Robot.driveBase.stopAutoMoveLeft();
		}

		// Shift the Robot right
		if (driveJoystick.getPovRight()) {
			Robot.driveBase.doAutoMoveRight();
		} else {
			Robot.driveBase.stopAutoMoveRight();
		}
        */
		
		// Log the Joystick X,Y Axis to the SmartDashboard.
		//SmartDashboard.putNumber("JoyStick Y Axis",FB);
		//SmartDashboard.putNumber("JoyStick X Axis",LR);

		SmartDashboard.putNumber("robotTurn",Robot.robotTurn);
		SmartDashboard.putNumber("robotDrive",Robot.robotDrive);

		if (Robot.isAutoBalance || 
		    Robot.isAutoClimbBalance || 
			Robot.isAutoMoveLeft ||
		    Robot.isAutoMoveRight) {
			// Don't do anything during auto commands
		} else if (Robot.targetType == Robot.targetTypes.TargetSeek) {
			// If we are seeking the throwing target, ignore the driver input
			Robot.driveBase.Drive(Robot.robotDrive,Robot.robotTurn);
		} else {
			// Set drivebase speed based on user input
			Robot.driveBase.Drive(FB,LR);
		}
	}

	/**********************************************************************************
	 * Returns true if command finished
	 **********************************************************************************/

	@Override
	public boolean isFinished() {
		return false;
	}

	/**********************************************************************************
	 * Called once after isFinished returns true
	 **********************************************************************************/

	 @Override
	public void end(boolean isInterrupted) {
	}  
}
