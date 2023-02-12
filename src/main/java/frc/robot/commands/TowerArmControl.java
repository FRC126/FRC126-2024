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
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

public class TowerArmControl extends CommandBase {
	JoystickWrapper operatorJoystick;
	
	/**********************************************************************************
	 **********************************************************************************/
	
    public TowerArmControl(TowerArm subsystem) {
		addRequirements(subsystem);
		operatorJoystick = new JoystickWrapper(Robot.oi.operatorController, 0.1);
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

		if (Robot.isAutoBalance || 
		    Robot.isAutoClimbBalance || 
			Robot.isAutoMoveLeft ||
		    Robot.isAutoMoveRight) {
		    return;
		}			

		// Get stick inputs
		double UD = operatorJoystick.getLeftStickY();
		
		if ( UD < .15 && UD > -0.15 ) {
			UD=0;
		}

		if ( operatorJoystick.isBackButton()) {
			Robot.robotTowerArm.resetEncoders();
		}

		// Log the Joystick X,Y Axis to the SmartDashboard.
		//SmartDashboard.putNumber("JoyStick A Button",operatorJoystick.isAButton());
		//SmartDashboard.putNumber("JoyStick X Axis",operatorJoystick.isBButton());

        Robot.robotTowerArm.MoveArm(UD);
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
        Robot.robotTowerArm.MoveArm(0);
	}  
    
}
