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

public class PickupControl extends CommandBase {
	JoystickWrapper operatorJoystick;
	
	/**********************************************************************************
	 **********************************************************************************/
	
    public PickupControl(Pickup subsystem) {
		addRequirements(subsystem);
		operatorJoystick = new JoystickWrapper(Robot.oi.operatorController, 0.1);
    }

	/**********************************************************************************
	 **********************************************************************************/
	
	@Override
	public void initialize() {
		Robot.robotPickup.RetractPickup();
	}    

	/**********************************************************************************
	 * Called every tick (20ms)
	 **********************************************************************************/
	
	@Override
	public void execute() {
		if (Robot.internalData.isAuto() || Robot.isAutoCommand) {
			// Ignore user controls during Autonomous
			return;
		}

		if (operatorJoystick.isLShoulderButton()) {
             // Deploy Pickup
             Robot.robotPickup.DeployPickup();
        } else if (operatorJoystick.isRShoulderButton()) {
             // Retract Pickup
             Robot.robotPickup.RetractPickup();
        } 

		if ( operatorJoystick.getPovUp() ) {
			Robot.robotPickup.pickupIntake();
		} else if (operatorJoystick.getPovDown()) {
			Robot.robotPickup.pickupEject();
		} else {
			Robot.robotPickup.cancel();
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
		Robot.robotPickup.RetractPickup();
	}     
}
