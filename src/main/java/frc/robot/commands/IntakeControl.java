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

public class IntakeControl extends CommandBase {
	static int delay=0;
    static boolean intakeExtended=false;
	static boolean autoIntakeExtend=false;
	static int intakeRPM=0;
	JoystickWrapper operatorJoystick;

	/**********************************************************************************
	 **********************************************************************************/
	
    public IntakeControl(BallIntake subsystem) {
		addRequirements(subsystem);
		operatorJoystick = new JoystickWrapper(Robot.oi.operatorController, 0.10);
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

		delay--;

		//////////////////////////////////////////////////////////////
		// Intake Controls

		// Extend the intake if the operator joystick left shoulder button is pressed
		if(operatorJoystick.isLShoulderButton()) {
			if (delay <= 0) {
                // Extend Ball Intake
                Robot.ballIntake.ExtendIntake();
                intakeExtended=true;
            }    
        }    

		// Retract the intake if the operator joystick right shoulder button is pressed
		if (operatorJoystick.isRShoulderButton()) {
            if (delay <= 0) {
                Robot.ballIntake.RetractIntake();
                intakeExtended=false;
            }    
        }
        
		// If the operator left trigger is pressed and held, the intake will extend
		// and run, when the X button is releasead the intake will stop and retract. 
		if(operatorJoystick.getLeftTrigger() > 0.1) {
			if (!autoIntakeExtend) {
                //Robot.ballIntake.ExtendIntake();
				autoIntakeExtend = true;
			} else {
				Robot.intakeRunning=true;
				Robot.ballIntake.IntakeRun();
     		}
		} else if (operatorJoystick.getRightTrigger() > 0.1) {
			if (!autoIntakeExtend) {
                //Robot.ballIntake.ExtendIntake();
				autoIntakeExtend = true;
			} else {
				Robot.intakeRunning=true;
				Robot.ballIntake.IntakeReverse();
     		}
		} else {
			if (autoIntakeExtend) {
				Robot.intakeRunning=false;
				Robot.ballIntake.IntakeStop();
                //Robot.ballIntake.RetractIntake();
			}

			// Read the operator right joystick and use the Y axis to run the intake 
			// Forwards or backwards.
			double foo = operatorJoystick.getRightStickY();
			if (foo > .2) {
				Robot.intakeRunning=true;
				Robot.ballIntake.IntakeRun();
			} else if ( foo < -.2) {
				Robot.intakeRunning=true;
				Robot.ballIntake.IntakeReverse();
			} else {
				Robot.intakeRunning=false;
				Robot.ballIntake.IntakeStop();
			}
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
