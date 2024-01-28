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
	int delay=0;

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

		SmartDashboard.putNumber("delay", delay);

		if (operatorJoystick.isBButton()) {
            // Toggle the thrower idle
			if (delay <= 0) {
				if (idleThrower) { idleThrower = false; } else { idleThrower = true; }
				delay=150;
			}
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

		SmartDashboard.putNumber("thrower myRPM", Robot.prototypeThrower.getRPM());
		SmartDashboard.putBoolean("idleThrower", idleThrower);

		if (operatorJoystick.isAButton()) {
			// Run the motors at 3800 rpm
			speed=Robot.prototypeThrower.getRPM();
		} else if (idleThrower) {
			// Idle the throwers
			speed=2000;
		} else {
			speed=0;
		}

     	SmartDashboard.putNumber("speed", speed);

		Robot.prototypeThrower.throwerRPM(1,speed);
		Robot.prototypeThrower.throwerRPM(2,speed);
	}

}

