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

public class ClimberCommand extends Command {
	JoystickWrapper operatorJoystick;

	/**********************************************************************************
	 **********************************************************************************/

	public ClimberCommand(Climber subsystem) {
		addRequirements(subsystem);
		operatorJoystick = new JoystickWrapper(Robot.oi.operatorController, 0.15);
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
    	// Climber Movement Control
		double y = operatorJoystick.getRightStickY();

		if (y > 0) {
	        Robot.climber.extendClimber(y);
		} else if (	y < 0 ) {
	        Robot.climber.retractClimber(y);
		} else {
			Robot.climber.cancel();
		}
	}
}
