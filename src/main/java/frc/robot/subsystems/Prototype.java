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

import com.ctre.phoenix6.*;

//import com.ctre.phoenix6.hardware.*;
//import frc.robot.RobotMap;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkRelativeEncoder;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**********************************************************************************
 **********************************************************************************/

public class Prototype extends SubsystemBase {	
	boolean protoDebug=true;
	
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

		/*
		///////////////////////////////////////////////////////////////
		// Falcom Motors

		Robot.protoTalonOne.set(speed);
		Robot.protoTalonTwo.set(speed);

		StatusSignal OneRPM = Robot.protoTalonOne.getVelocity();
		StatusSignal TwoRPM = Robot.protoTalonTwo.getVelocity();

		motorOneRPM = OneRPM.getValueAsDouble() * 60;
		motorTwoRPM = TwoRPM.getValueAsDouble() * 60 ;
		*/
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

