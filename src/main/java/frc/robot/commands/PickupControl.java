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

public class PickupControl extends Command {
	JoystickWrapper operatorJoystick;
	Pickup pickup;

	/**********************************************************************************
	 **********************************************************************************/

	public PickupControl(Pickup subsystem) {
		addRequirements(subsystem);
		operatorJoystick = new JoystickWrapper(Robot.oi.operatorController, 0.15);
		this.pickup = subsystem;
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

		if (Robot.internalData.isAuto() || Robot.isAutoCommand) {
			// Ignore user controls during Autonomous
			return;
		}		

		if (operatorJoystick.isLShoulderButton() || operatorJoystick.isYButton()) {
			this.pickup.pickupMotorOn();
		} else if (operatorJoystick.isRShoulderButton()) {
			this.pickup.pickupMotorOff();
		} else {
			this.pickup.cancel();
		}
	}
}
