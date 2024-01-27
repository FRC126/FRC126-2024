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

public class Prototype extends SubsystemBase {	
	boolean protoDebug=false;
	
	/************************************************************************
	 ************************************************************************/

	public Prototype() {

		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new PrototypeControl(this));
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

    /************************************************************************
	 ************************************************************************/

	public void runMotors(double speed) {
		double motorOneRPM, motorTwoRPM;

		///////////////////////////////////////////////////////////////
		//Spark max/Neo Motors

		Robot.ProtoMotorOne.set(speed);
		Robot.ProtoMotorTwo.set(speed);

		motorOneRPM = Math.abs(Robot.ProtoMotorOneRelativeEncoder.getVelocity());
		motorTwoRPM = Math.abs(Robot.ProtoMotorTwoRelativeEncoder.getVelocity());
		
		if (protoDebug) {
			SmartDashboard.putNumber("Proto One RPM",motorOneRPM);
			SmartDashboard.putNumber("Proto Two RPM",motorTwoRPM);
		}
		
	}

    /************************************************************************
	 ************************************************************************/

	public void cancel() {
        runMotors(0); 
	}
}

