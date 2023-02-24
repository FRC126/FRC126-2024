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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;

/**********************************************************************************
 **********************************************************************************/

public class TowerArm extends SubsystemBase {
	double lastSpeed=1000;
	int limitHit=0;

	/************************************************************************
	 ************************************************************************/

	public TowerArm() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new TowerArmControl(this));

		// Set brake mode for the tower arm motor
		Robot.TowerArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

	/************************************************************************
	 ************************************************************************/

	public void MoveArm(double speedIn) { 
		double speed = speedIn;

		//Check encoders to if we are at limits.
		double pos = getPos();

		SmartDashboard.putNumber("Tower Arm Pos", pos);

		if (Robot.towerArmRetracedLimit.get() == false) {
			SmartDashboard.putBoolean("Tower Arm Limit", true);
			limitHit=0;	 
		} else {
		    SmartDashboard.putBoolean("Tower Arm Limit", false);
			if (speed > 0) { speed=0; }
			limitHit++;
			if (limitHit > 20) {
  			     Robot.GrabberRelativeEncoder.setPosition(5);
				 limitHit=0;	 
			}
		}

		if (speed != 0) {	
			if (speed < 0) { 
				if (pos < RobotMap.towerArmRetractedPos + 10 && !Robot.ignoreEncoders) { speed = -.1; }
				if (pos < RobotMap.towerArmRetractedPos && !Robot.ignoreEncoders) { speed = 0; }
			}

			if (speed > 0) { 
				if (pos > RobotMap.towerArmExtendedMaxPos - 10 && !Robot.ignoreEncoders) { speed = .1; }
				if (pos > RobotMap.towerArmExtendedMaxPos && !Robot.ignoreEncoders) { speed = 0; }
			}
        }

		if (speed != lastSpeed) {
			// Send power to the drive motors
			Robot.TowerArmMotor.set(speed * RobotMap.TowerArmMotorInversion);
			lastSpeed = speed;

			SmartDashboard.putNumber("Tower Arm Speed", speed);
		}	

		if (speed == 0 && lastSpeed != 0) {
			Robot.TowerArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		}
	}

    /************************************************************************
	 ************************************************************************/

	public void resetEncoders() {
		// Need to use encoders for the NEOs
		Robot.TowerArmRelativeEncoder.setPosition(0);
	}

    /************************************************************************
	 *************************************************************************/

	 public void cancel() {
        MoveArm(0);
	}

    /************************************************************************
	 ************************************************************************/

	public double getPos() {
		return(Robot.TowerArmRelativeEncoder.getPosition());
	}

}