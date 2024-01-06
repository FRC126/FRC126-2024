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
		if (operatorJoystick.isXButton()) {
			Robot.stopAutoCommand();
		}
		
		if (Robot.internalData.isAuto() || Robot.isAutoCommand) {
			// Ignore user controls during Autonomous
			return;
		}

		// Get stick inputs
		double UD = operatorJoystick.getLeftStickY();
		
		if ( UD < .15 && UD > -0.15 ) {
			UD=0;
		}

		if ( operatorJoystick.isAButton() ) {
			if ( Robot.doAutoCommand() ) {
     		    Robot.autoCommand=new AutoPlaceConeLow(0);
					Robot.autoMove=true;
				Robot.autoCommand.schedule();
			};
		} else if ( operatorJoystick.isBButton() ) {
			if ( Robot.doAutoCommand() ) {
				//if ( operatorJoystick.getPovLeft() ) {
		 			Robot.autoCommand=new AutoShelf();
					Robot.autoMove=false;
					Robot.autoCommand.schedule();
				//} else {
				//	Robot.autoCommand=new AutoPlaceConeMid(0);
				//	Robot.autoMove=true;
				//}	
				Robot.autoCommand.schedule();
			};
		} else if ( operatorJoystick.isYButton() ) {
			if ( Robot.doAutoCommand() ) {
				Robot.autoCommand=new AutoPlaceConeHigh(0);
				Robot.autoMove=true;
				Robot.autoCommand.schedule();
			};
		} 

		if ( operatorJoystick.isBackButton() ) {
			Robot.ignoreEncoders=true;
		} else {
			Robot.ignoreEncoders=false;
		}	

		if ( operatorJoystick.isStartButton()) {
			Robot.robotTowerArm.resetEncoders();
			Robot.robotArmExtension.resetEncoders();
			Robot.robotGrabber.resetEncoders();
			Robot.robotCatapult.resetEncoders();
		}
	
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
