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
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

public class Grabber extends SubsystemBase {

	public static double grabberClosedPos=0;
	public static double grabberOpenPos=331;
	public static double grabberConePos=50;
	public static double grabberCubePos=280;
	
	/************************************************************************
	 ************************************************************************/

	public Grabber() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new GrabberControl(this));

		resetEncoders();

		// Brake mode on for the motor
		Robot.GrabberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

	/************************************************************************
	 ************************************************************************/

	 /************************************************************************
	 * Send power to the drive motors
	 ************************************************************************/

	public void MoveGrabber(double speedIn) { 
		double speed = speedIn;
		
        //  Check encoders to if we are at limits.
		double pos=getPos();
		
		if ( speed > 0) { 
			if (pos >= grabberOpenPos && !Robot.ignoreEncoders) { speed = 0; }
		}

		if ( speed < 0) { 
			if (pos <= grabberClosedPos && !Robot.ignoreEncoders) { speed = 0; }
		}

		SmartDashboard.putNumber("Grabber Pos", pos);
		SmartDashboard.putNumber("Grabber Speed", speed);

		Robot.GrabberMotor.set(speed * RobotMap.GrabberMotorInversion);

		if (speed == 0) {
			Robot.GrabberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		}
	}

    /************************************************************************
	 ************************************************************************/

	public void resetEncoders() {
		// Need to use encoders for the NEOs
		Robot.GrabberRelativeEncoder.setPosition(0);
	}

    /************************************************************************
	 ************************************************************************/

	 public double getPos() {
		// Need to use encoders for the NEOs
		return(Robot.GrabberRelativeEncoder.getPosition()*-1);
	}

	/************************************************************************
	 *************************************************************************/

	 public void cancel() {
        MoveGrabber(0); 
	}

}