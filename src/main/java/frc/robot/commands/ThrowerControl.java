/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2024 Code       
	Go get em gaels!

***********************************/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.*;
import frc.robot.JoystickWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ThrowerControl extends Command {
	JoystickWrapper operatorJoystick;
	boolean idleThrower=false;
	boolean runThrower=false;
	int delay=0;
	int runCount=0;
	final int IDLE_RPM=2000;
	final int MAX_RPM=5700;
	boolean throwerTriggered=false;

	/**********************************************************************************
	 **********************************************************************************/
	
    public ThrowerControl(Thrower subsystem) {
		addRequirements(subsystem);
		operatorJoystick = new JoystickWrapper(Robot.oi.operatorController	, 0.15);
    }

	/**********************************************************************************
	 **********************************************************************************/
	
	@Override
	public void initialize() {
		Robot.stopAutoCommand();
	}    

	/**********************************************************************************
	 * Called every tick (20ms)
	 **********************************************************************************/
	
	@Override
	public void execute() {
		double speed;

		if ( delay > 0) { delay--; }
		if ( runCount > 0) { runCount--; }

		SmartDashboard.putNumber("delay", delay);

		if (operatorJoystick.isBButton()) {
            // Toggle the thrower idle on and off
			if (delay <= 0) {
				if (idleThrower) { idleThrower = false; } else { idleThrower = true; }
				delay=150;
			}
		}	
/* 
		// Press Y to run the thrower for 10 seconds 
		if (operatorJoystick.isYButton()) {
			if (delay <= 0) {
				if (runThrower) { 
					runThrower = false; 
					runCount = 0;
				} else { 
					runThrower = true; 
					runCount = 500;
				}
				delay=40;
			}
		}
		if (runCount == 0) {
			runThrower = false;
		}
*/
		if (operatorJoystick.getPovLeft()) {
			if (delay <= 0) {
   			    Robot.thrower.setRPM(Robot.thrower.getRPM()-100);
				delay=25;
			}	
		} 	
		if (operatorJoystick.getPovRight()) {
			if (delay <= 0) {
                Robot.thrower.setRPM(Robot.thrower.getRPM()+100);
				delay=25;
			}	
		} 	
		if (operatorJoystick.getPovUp()) {
			if (delay <= 0) {
   			    Robot.thrower.setRPM(Robot.thrower.getRPM()+1000);
				delay=25;
			}	
		} 	
		if (operatorJoystick.getPovDown()) {
			if (delay <= 0) {
                Robot.thrower.setRPM(Robot.thrower.getRPM()-1000);
				delay=25;
			}	
		} 	
		if (Robot.thrower.getRPM() < 0) {
			Robot.thrower.setRPM(0);
		}
		if (Robot.thrower.getRPM() > MAX_RPM) {
			Robot.thrower.setRPM(MAX_RPM);
		}

		SmartDashboard.putNumber("thrower myRPM", Robot.thrower.getRPM());
		SmartDashboard.putBoolean("idleThrower", idleThrower);

		if (operatorJoystick.isAButton()) {
			// double distance=Robot.distance.getDistanceAvg();
			// TODO set the thrower angle and speed based on the distance
			// TODO TODO TODO

    		// Run the motors at specified rpm
			speed=Robot.thrower.getRPM();
		} else if (idleThrower) {
			// Idle the throwers
			speed=IDLE_RPM;
			throwerTriggered=false;
		} else if (runThrower) {
			speed=Robot.thrower.getRPM();
			throwerTriggered=false;
		} else {
			speed=0;
			throwerTriggered=false;
		}

     	SmartDashboard.putNumber("speed", speed);

		int reachedOne=0, reachedTwo=0;
		reachedOne = Robot.thrower.throwerRPM(1,speed);
		reachedTwo = Robot.thrower.throwerRPM(2,speed);

		SmartDashboard.putNumber("reachedOne", reachedOne);
     	SmartDashboard.putNumber("reachedTwo", reachedTwo);

		// If we have reached the target rpm on the thrower, run the trigger and shoot the note
		if ((reachedOne > 2 && reachedTwo > 2 && operatorJoystick.isAButton()) || operatorJoystick.isXButton()) {
            Robot.thrower.throwerTriggerOn();
			throwerTriggered=true;

		} else {
			if (!throwerTriggered) Robot.thrower.throwerTriggerOff();
		}

        // Thrower Angle Control
		double y = operatorJoystick.getLeftStickY();

		SmartDashboard.putNumber("thrower tilt input", y);

		if ( y!=0 ) {
			Robot.thrower.moveThrower(y);
		} else {
			Robot.thrower.moveThrower(0);
		}
	}
}

