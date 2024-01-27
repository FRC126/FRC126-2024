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

		if (operatorJoystick.isBButton()) {
            // Toggle the thrower idle
			if (delay <= 0) {
				if (idleThrower) { idleThrower = false; } else { idleThrower = true; }
				delay=150;
			}
		}	

		if (operatorJoystick.isAButton()) {
			// Run the motors at 3800 rpm
			speed=3800;
		} else if (idleThrower) {
			// Idle the throwers
			speed=1500;
		} else {
			speed=0;
		}

		Robot.prototypeThrower.throwerRPM(1,speed);
		Robot.prototypeThrower.throwerRPM(2,speed);
	}

}

