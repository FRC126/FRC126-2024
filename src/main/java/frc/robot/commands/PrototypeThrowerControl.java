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


public class PrototypeThrowerControl extends Command {
	JoystickWrapper operatorJoystick;
	boolean idleThrower=false;
	boolean runThrower=false;
	int delay=0;
	int runCount=0;

	/**********************************************************************************
	 **********************************************************************************/
	
    public PrototypeThrowerControl(PrototypeThrower subsystem) {
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
            // Toggle the thrower idle
			if (delay <= 0) {
				if (idleThrower) { idleThrower = false; } else { idleThrower = true; }
				delay=150;
			}
		}	
		if (operatorJoystick.isYButton()) {
            // Toggle the thrower idle
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

		if (operatorJoystick.getPovLeft()) {
			if (delay <= 0) {
   			    Robot.prototypeThrower.setRPM(Robot.prototypeThrower.getRPM()-100);
				delay=25;
			}	
		} 	
		if (operatorJoystick.getPovRight()) {
			if (delay <= 0) {
                Robot.prototypeThrower.setRPM(Robot.prototypeThrower.getRPM()+100);
				delay=25;
			}	
		} 	
		if (operatorJoystick.getPovUp()) {
			if (delay <= 0) {
   			    Robot.prototypeThrower.setRPM(Robot.prototypeThrower.getRPM()+1000);
				delay=25;
			}	
		} 	
		if (operatorJoystick.getPovDown()) {
			if (delay <= 0) {
                Robot.prototypeThrower.setRPM(Robot.prototypeThrower.getRPM()-1000);
				delay=25;
			}	
		} 	
		if (Robot.prototypeThrower.getRPM() < 0) {
			Robot.prototypeThrower.setRPM(0);
		}
		if (Robot.prototypeThrower.getRPM() > 5700) {
			Robot.prototypeThrower.setRPM(5700);
		}

		SmartDashboard.putNumber("thrower myRPM", Robot.prototypeThrower.getRPM());
		SmartDashboard.putBoolean("idleThrower", idleThrower);

		if (operatorJoystick.isAButton()) {
			// Run the motors at 3800 rpm
			speed=Robot.prototypeThrower.getRPM();
		} else if (idleThrower) {
			// Idle the throwers
			speed=2000;
		} else if (runThrower) {
			speed=Robot.prototypeThrower.getRPM();
		} else {
			speed=0;
		}

     	SmartDashboard.putNumber("speed", speed);

		Robot.prototypeThrower.throwerRPM(1,speed);
		Robot.prototypeThrower.throwerRPM(2,speed);
	}

}

