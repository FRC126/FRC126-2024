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

	double lastSpeed=1000;
	int limitHit=0;
	double softSpeed=0;
	
	/************************************************************************
	 ************************************************************************/

	public Grabber() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new GrabberControl(this));

		// Brake mode on for the motor
		brakesOn();	
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
		double pos = getPos();

		SmartDashboard.putNumber("Grabber Pos", pos);

		if (Robot.grabberRetracedLimit.get() == false) {
			SmartDashboard.putBoolean("Grabber Limit", true);
			limitHit=0;	 
		} else {
		    SmartDashboard.putBoolean("Grabber Limit", false);
			if (speed > 0) { speed=0; }
			limitHit++;
			if (limitHit > 20) {
  			     Robot.GrabberRelativeEncoder.setPosition(5);
				 limitHit=0;	 
			}
		}
		
		// Soft start code
		if (speed == 0) {
			// No movement
		    softSpeed = 0;
		} else if (speed > 0) {
			// Soft start for arm up
			if ( speed > softSpeed) {
				speed = softSpeed + 0.05;
			}			
			softSpeed=speed;
		} else {
			// Soft start for throttle reverse
			if ( speed < softSpeed) {
				speed = softSpeed - 0.05;
			}			
			softSpeed=speed;
		}	

		if (speed != 0) {	
			if (speed < 0) { 
				if (pos >= RobotMap.grabberOpenPos - 20 && !Robot.ignoreEncoders) { speed = -.2; }
				if (pos >= RobotMap.grabberOpenPos && !Robot.ignoreEncoders) { speed = 0; }
			}

			if (speed > 0) { 
				if (pos <= RobotMap.grabberClosedPos + 20 && !Robot.ignoreEncoders) { speed = .2; }
				if (pos <= RobotMap.grabberClosedPos && !Robot.ignoreEncoders) { speed = 0; }
			}
		}

		double cur=Robot.GrabberMotor.getOutputCurrent();
		SmartDashboard.putNumber("Grabber Current", cur);

		if (speed != lastSpeed) {
			Robot.GrabberMotor.set(speed * RobotMap.GrabberMotorInversion);
			lastSpeed = speed;

			//SmartDashboard.putNumber("Grabber Speed", speed);
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
		return(Robot.GrabberRelativeEncoder.getPosition() * -1);
	}

	/************************************************************************
	 *************************************************************************/

	 public void cancel() {
        MoveGrabber(0); 
	}

	/************************************************************************
	 *************************************************************************/

	 public void brakesOn() {
		Robot.GrabberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

    /************************************************************************
	 ************************************************************************/

	 public void brakesOff() {
		Robot.GrabberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}	

}