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

public class ThrowerControl extends CommandBase {
	static double speed;
	static int delay=0;
	static int throwCount=0;
    static int throwerRPM=0;
	static boolean autoThrow=false;
	JoystickWrapper operatorJoystick;
	static int targetReachedCount=0;

	/**********************************************************************************
	 **********************************************************************************/
	
    public ThrowerControl(BallThrower subsystem) {
		addRequirements(subsystem);
		operatorJoystick = new JoystickWrapper(Robot.oi.operatorController, 0.05);
	}

	/**********************************************************************************
	 **********************************************************************************/
	
	public void initialize() {
	}    

	/**********************************************************************************
	 * Called every tick (20ms)
	 **********************************************************************************/
	
	@SuppressWarnings("static-access")
	@Override
	public void execute() {
		if (Robot.internalData.isAuto() || Robot.isThrowCommand ) {
			// Ignore user controls during Autonomous
			return;
		}

		// Use the POV pad to set 4 different throwing distances.  Button needs to be
		// held down for length of throwing action
		if (operatorJoystick.getPovDown()) {
            throwerRPM=13000;
			autoThrow=true;
		} else if (operatorJoystick.getPovLeft()) {
			throwerRPM=14000;
			autoThrow=true;
		} else if (operatorJoystick.getPovUp()) {
			throwerRPM=15000;
			autoThrow=true;
		} else if (operatorJoystick.getPovRight()) {
			throwerRPM=7000;
			autoThrow=true;
		} else {
			if ( autoThrow == true ) {
				// IF autoThrow was true, cancel it.
				throwerRPM=0;
				autoThrow=false;
				Robot.throwerRunning=false;
				Robot.ballThrower.ThrowerIntakeStop();
			}
			targetReachedCount=0;
		}	

		// Manually increment the throwing wheel speeds
        if (operatorJoystick.isAButton()) {
            // Run Ball Intake
		    if (delay <= 0) {
				throwerRPM+=500;
				delay=5;
			}	
        } 

		// Manually decrement the throwing wheel speeds
		if (operatorJoystick.isBButton()) {
            // Run Ball Intake
		    if (delay <= 0) {
				throwerRPM-=500;
				delay=5;
			}	
        } 

		// Stop the throwing wheels
		if (operatorJoystick.isYButton()) {
            // Run Ball Intake
		    throwerRPM=0;
        } 

        // Range check the RPM
		if (throwerRPM > 20500) { throwerRPM = 20500; }
		if (throwerRPM < 0) { throwerRPM = 0; }

		// Call throwerRPM to set the target RPM for the thrower motors and check if 
		// the target RPM has been reached
		boolean rpmReached = Robot.ballThrower.throwerRPM(throwerRPM);

		if (rpmReached) {
			targetReachedCount++;
		 } else {
			//targetReachedCount=0;
		}

		if (rpmReached && autoThrow && targetReachedCount > 10) {
			// If we reached the target RPM, and autoThrow is set, run the thrower intake motor
			Robot.ballThrower.ThrowerIntakeRun();
			Robot.throwerRunning=true;
		} else {
			if (targetReachedCount > 10 && autoThrow ) {
				Robot.throwerRunning=true;
				Robot.ballThrower.ThrowerIntakeRun();
		    } else {
				// If autoThrow is false or the targetRPM isn't reach, run the thrower intake motor
				// motor is the X button is pressed, otherwise stop the thrower intake motor
				if (operatorJoystick.isXButton()) {
					Robot.throwerRunning=true;
					Robot.ballThrower.ThrowerIntakeRun();
				} else {
					Robot.throwerRunning=false;
					Robot.ballThrower.ThrowerIntakeStop();
				}
			}		
		}

		delay--;	
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
