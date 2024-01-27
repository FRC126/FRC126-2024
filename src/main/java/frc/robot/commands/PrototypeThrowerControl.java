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
        double y1 = operatorJoystick.getRightStickY();

		if (operatorJoystick.isAButton()) {
			Robot.prototypeThrower.throwerRPM(1,3800);
			Robot.prototypeThrower.throwerRPM(2,3800);
		} else if (y1 > 0) {
            Robot.prototypeThrower.runMotors(y1);
		} else {
			// Idle the throwers
			Robot.prototypeThrower.throwerRPM(1,1500);
			Robot.prototypeThrower.throwerRPM(2,1500);
		}
	}

}

