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

public class PickupControl extends Command {
	JoystickWrapper operatorJoystick;
	boolean idleThrower = false;
	boolean runThrower = false;
	int delay = 0;
	int runCount = 0;
	boolean runPickup = false;
	private static int DUMMY = 0;
	boolean increasing = true;

	/**********************************************************************************
	 **********************************************************************************/

	public PickupControl(Pickup subsystem) {
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

		if (operatorJoystick.isXButton()) {
			increasing = !increasing;
		}
		int value = DUMMY;
		if (increasing) {
			value = value + 1;
		} else {
			value = value - 1;
		}
		DUMMY = value;
		SmartDashboard.putNumber("DUMMY", DUMMY);

	}
}
