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

public class Pickup extends SubsystemBase {	
    boolean pickupDebug=false;
	double pickupRPM;

	/************************************************************************
	 ************************************************************************/

	public Pickup() {

		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new PickupControl(this));
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

	/************************************************************************
     * Run Main Thower Wheels by target RPM
	 ************************************************************************/

	 public void runMotors(double speed) {

		///////////////////////////////////////////////////////////////
		//Spark max/Neo Motors

		Robot.PickupMotor.set(speed);

		pickupRPM = Math.abs(Robot.PickupMotorEncoder.getVelocity());
		setRPM(pickupRPM);

		if (pickupDebug) {
			SmartDashboard.putNumber("Pickup Motor RPM",pickupRPM);
		}
		
	}

    /************************************************************************
	 ************************************************************************/

	public void cancel() {
        runMotors(0); 
	}

	public double getRPM() {
		return pickupRPM;
	}

	public void setRPM(double rpmIn) {
		pickupRPM = rpmIn;
	}
}


