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
import com.revrobotics.CANSparkMax;

/**********************************************************************************
 **********************************************************************************/

public class DriverControl extends CommandBase {
	JoystickWrapper driveJoystick;
	
	/**********************************************************************************
	 **********************************************************************************/
	
    public DriverControl(WestCoastDrive subsystem) {
		addRequirements(subsystem);
		driveJoystick = new JoystickWrapper(Robot.oi.driveController, 0.1);
    }

	/**********************************************************************************
	 **********************************************************************************/
	
	@Override
	public void initialize() {
	}    

	/**********************************************************************************
	 * Called every tick (20ms)
	 **********************************************************************************/
	
	@SuppressWarnings("static-access")
	@Override
	public void execute() {
		if (Robot.internalData.isAuto()) {
			// Ignore user controls during Autonomous
			return;
		}

		// Get stick inputs
        double FB = driveJoystick.getLeftStickY();
        double LR = driveJoystick.getRightStickX();

		if (driveJoystick.getLeftTrigger() > .3) {
			LR=LR*.7;
			FB=FB*.3;
		}

		if (driveJoystick.getRightTrigger() > .3) {
			Robot.driveBase.brakesOn();
		} else {
			Robot.driveBase.brakesOff();
		}	

		//if (driveJoystick.isRStickPressButton()) {
		//	LR = LR *.5;
		//}
		//if (driveJoystick.isLStickPressButton()) {
		//	FB = FB *.5;
		//}

		if ( FB > .2 || FB < -0.2 || LR > .2 || LR < -0.2 ) {
			// Turn off the brakes if operator is using the joysticks
			Robot.driveBase.brakesOff();
		}

		if (driveJoystick.isAButton()) {
			Robot.driveBase.doAutoBalance();
		} else {
			Robot.driveBase.stopAutoBalance();
		}

		// Log the Joystick X,Y Axis to the SmartDashboard.
		//SmartDashboard.putNumber("JoyStick Y Axis",FB);
		//SmartDashboard.putNumber("JoyStick X Axis",LR);

		SmartDashboard.putNumber("robotTurn",Robot.robotTurn);
		SmartDashboard.putNumber("robotDrive",Robot.robotDrive);

		if (Robot.isAutoBalance) {
			// Don't do anything during autobalance
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
