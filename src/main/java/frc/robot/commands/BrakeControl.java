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

public class BrakeControl extends CommandBase {
	JoystickWrapper driverJoystick;
	
	/**********************************************************************************
	 **********************************************************************************/
	
    public BrakeControl(Brakes subsystem) {
		addRequirements(subsystem);
		driverJoystick = new JoystickWrapper(Robot.oi.driveController, 0.1);
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
		if (Robot.internalData.isAuto() || Robot.isAutoCommand) {
			// Ignore user controls during Autonomous
			return;
		}

        if (driverJoystick.isLShoulderButton()) {
             // Deploy Brakes
             Robot.robotBrakes.ApplyBrakes();
        } else if (driverJoystick.isRShoulderButton()) {
             // Retract Brakes
             Robot.robotBrakes.ReleaseBrakes();
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
        // TODO Retract Brakes
	}     
}
