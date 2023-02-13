/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2023 Code       
	Go get em gaels!

***********************************/

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

public class Grabber extends SubsystemBase {

	/************************************************************************
	 ************************************************************************/

	public Grabber() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new GrabberControl(this));

		resetEncoders();

		// Do we want brake mode on for the motors?
		//Robot.GrabberMotor.setNeutralMode(NeutralMode.Brake);
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

	/************************************************************************
	 ************************************************************************/

	 /************************************************************************
	 * Send power to the drive motors
	 ************************************************************************/

	public void MoveGrabber(double speed) { 

		if (Robot.internalData.isTeleop()) {
    		// Slow down the turning in teleop
		}

		//SmartDashboard.putNumber("arm speed", speed);

        //TODO Check encoders to if we are at limits.

		Robot.GrabberMotor.set(speed * RobotMap.GrabberMotorInversion);
	}

    /************************************************************************
	 ************************************************************************/

	public void resetEncoders() {
		// Need to use encoders for the NEOs
		Robot.GrabberRelativeEncoder.setPosition(0);
	}

	/************************************************************************
	 *************************************************************************/

	 public void cancel() {
        MoveGrabber(0); 
	}

}