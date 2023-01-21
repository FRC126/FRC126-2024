/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2022 Code       
	Go get em gaels!

***********************************/

package frc.robot.commands;

import frc.robot.JoystickWrapper;
import frc.robot.Robot;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**********************************************************************************
 **********************************************************************************/

public class ClimberControl extends CommandBase {
    static boolean intakeExtended=false;
	static int intakeRPM=0;
	JoystickWrapper driveJoystick;

	/**********************************************************************************
	 **********************************************************************************/
	
    public ClimberControl(VerticalClimber subsystem) {
		addRequirements(subsystem);
		driveJoystick = new JoystickWrapper(Robot.oi.driveController, 0.05);
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

		//////////////////////////////////////////////////////////////
		// Climber Controls

		Robot.verticalClimber.getRightPos();
		Robot.verticalClimber.getLeftPos();
		
        if (driveJoystick.isAButton()) {
            // Extend the Climber while the A Button is pressed
		    Robot.verticalClimber.RaiseClimber();
        }  else if (driveJoystick.isBButton()) {
            // Retact the Climber while the B Button is pressed
		    Robot.verticalClimber.LowerClimber();
        } else if (driveJoystick.getPovLeft()) {
			// Lower just the left climber, don't stop at 0
		    Robot.verticalClimber.LowerLeftClimber(false);
        } else if (driveJoystick.getPovRight()) {
			// Lower just the right climber, don't stop at 0
		    Robot.verticalClimber.LowerRightClimber(false);
		} else {
			// If none of the climber buttons are pressed, stop moving the climber
		    Robot.verticalClimber.StopClimber();
		}

		if(driveJoystick.isStartButton()) {
			// Reset the climber encoders
			Robot.verticalClimber.ResetClimberEncoder();
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
