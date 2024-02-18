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
		double pickMotorSpeed = SmartDashboard.getNumber(Robot.PICKUP_MOTOR_SPEED_STRING, 1.0);
		if (pickMotorSpeed > 1.0) {
			pickMotorSpeed = 1.0;
		} else if (pickMotorSpeed < -1.0) {
			pickMotorSpeed = -1.0;
		}

		if (operatorJoystick.isYButton()) {
			this.pickup.runMotor(pickMotorSpeed*-1);
     		SmartDashboard.putNumber("pickup run speed", pickMotorSpeed*-1);
		} else {
			this.pickup.cancel();
		}
	}
}
