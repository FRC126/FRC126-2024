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

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

public class PickupSubsystem extends SubsystemBase {
	boolean pickupDebug = false;
	double pickupRPM;
	int called = 0;

	/************************************************************************
	 ************************************************************************/

	public PickupSubsystem() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new PickupCommand(this));
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {
	}

	/************************************************************************
	 * Run Main Thower Wheels by target RPM
	 ************************************************************************/

	public void runMotor(double speed) {
		///////////////////////////////////////////////////////////////
		// Spark max/Neo Motors

		Robot.PickupMotor.set(speed);
		//SmartDashboard.putNumber("Pickup runMotor called", called++);
	}

	/************************************************************************
	 ************************************************************************/

	public void cancel() {
		runMotor(0);
	}
}
