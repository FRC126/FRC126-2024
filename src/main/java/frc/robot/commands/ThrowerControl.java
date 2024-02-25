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
	boolean throwerDebug=true;
	int delay=0;
	final int IDLE_RPM=2000;
	final int MAX_RPM=4800;

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

		if (Robot.internalData.isAuto() || Robot.isAutoCommand) {
			// Ignore user controls during Autonomous
			return;
		}	
		
		if ( delay > 0) { delay--; }

		// Thrower Angle Control
		double y = operatorJoystick.getLeftStickY();

		if (operatorJoystick.isStartButton()) {
			Robot.thrower.resetEncoders();
		};

  	    if (operatorJoystick.getPovUp()) {
   		    Robot.thrower.setThrowerPosition(100);
		} else if (operatorJoystick.getPovRight()) {
   		    Robot.thrower.setThrowerPosition(45);
		} else if (operatorJoystick.getPovDown()) {
   		    Robot.thrower.setThrowerPosition(30);
		} else if (operatorJoystick.getPovLeft()) {
   		    Robot.thrower.setThrowerPosition(145);
		} else {
			if ( y!=0 ) {
				Robot.thrower.moveThrower(y);
			} else {
				if ( !Robot.thrower.getAutoMoveThrower()) {
				    Robot.thrower.moveThrower(0);
				}	
			}			
		}

		if (operatorJoystick.isBButton()) {
            // Toggle the thrower idle on and off
			if (delay <= 0) {
				if (idleThrower) { idleThrower = false; } else { idleThrower = true; }
				delay=30;
			}
		}	

		if (operatorJoystick.getRightStickY()<-.25) {
			if (delay <= 0) {
   			    Robot.thrower.setRPM(Robot.thrower.getRPM()-100);
				delay=15;
			}	
		} 	
		if (operatorJoystick.getRightStickY()>.25) {
			if (delay <= 0) {
                Robot.thrower.setRPM(Robot.thrower.getRPM()+100);
				delay=15;
			}	
		} 	
		if (Robot.thrower.getRPM() < 0) {
			Robot.thrower.setRPM(0);
		}
		if (Robot.thrower.getRPM() > MAX_RPM) {
			Robot.thrower.setRPM(MAX_RPM);
		}

		if (operatorJoystick.isAButton() || operatorJoystick.rightTriggerPressed()) {
			// double distance=Robot.distance.getDistanceAvg();

			// TODO set the thrower angle and speed based on the distance
			// TODO TODO TODO

    		// Run the motors at specified rpm
			speed=Robot.thrower.getRPM();
			Robot.Leds.setMode(LEDSubsystem.LEDModes.ShootingSpeaker);
		} else if (idleThrower) {
			// Idle the throwers
			speed=IDLE_RPM;
			Robot.thrower.setThrowTriggered(false);
		} else {
			speed=0;
			Robot.thrower.setThrowTriggered(false);
		}
		int reachedOne=0, reachedTwo=0;
		if (!operatorJoystick.rightTriggerPressed()){
			reachedOne = Robot.thrower.throwerRPM(1,speed);
			reachedTwo = Robot.thrower.throwerRPM(2,speed);
		}

		if (throwerDebug) {
			SmartDashboard.putNumber("thrower myRPM", Robot.thrower.getRPM());
			SmartDashboard.putBoolean("idleThrower", idleThrower);
			SmartDashboard.putNumber("reachedOne", reachedOne);
    	 	SmartDashboard.putNumber("reachedTwo", reachedTwo);
	     	SmartDashboard.putNumber("speed", speed);
			SmartDashboard.putNumber("thrower tilt input", y);
		}

		// If we have reached the target rpm on the thrower, run the trigger and shoot the note
		if ((reachedOne > 2 && reachedTwo > 2 && 
		      (operatorJoystick.isAButton() || operatorJoystick.rightTriggerPressed())) || 
			  operatorJoystick.isXButton()) {
            Robot.thrower.throwerTriggerOn();
			Robot.thrower.setThrowTriggered(true);
		} else if (operatorJoystick.isLShoulderButton()) {
			Robot.thrower.throwerTriggerReverse();
			Robot.thrower.setThrowerSpeed(-1.0);
		} else if (!Robot.thrower.getThrowTriggered() && !Robot.thrower.getAutoTriggerRun()) {
			Robot.thrower.throwerTriggerOff();
		} 
	}
}

