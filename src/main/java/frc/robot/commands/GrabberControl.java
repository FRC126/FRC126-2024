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

public class GrabberControl extends CommandBase {
	JoystickWrapper driveJoystick;
	
	/**********************************************************************************
	 **********************************************************************************/
	
    public GrabberControl(Grabber subsystem) {
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
        double speed=0;

		if (Robot.internalData.isAuto()) {
			// Ignore user controls during Autonomous
			return;
		}

     	if (driveJoystick.isAButton()) {
            speed=0.3;
			// Arm Up
		} else if (driveJoystick.isBButton()) {
            speed=-0.3;
			// Arm Down
		} else {
            // Arm Stop
            speed=0;
        }

		// Log the Joystick X,Y Axis to the SmartDashboard.
		//SmartDashboard.putNumber("JoyStick A Button",driveJoystick.isAButton());
		//SmartDashboard.putNumber("JoyStick X Axis",driveJoystick.isBButton());

        Robot.robotArm.MoveArm(speed);
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
