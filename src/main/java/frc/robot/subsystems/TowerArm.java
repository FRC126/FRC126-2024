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
	double softSpeed=0;

	/************************************************************************
	 ************************************************************************/

	public TowerArm() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new TowerArmControl(this));

		// Set brake mode for the tower arm motor
		brakesOn();
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

		if (Robot.towerArmRetracedLimit.get() != false) {
			SmartDashboard.putBoolean("Tower Arm Limit", true);
			limitHit=0;	 
		} else {
		    SmartDashboard.putBoolean("Tower Arm Limit", false);
			if (speed < 0) { speed=0; }
			limitHit++;
			if (limitHit > 10) {
  			     Robot.TowerArmRelativeEncoder.setPosition(5);
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
				speed = softSpeed + 0.1;
			}			
			softSpeed=speed;
		} else {
			// Soft start for throttle reverse
			if ( speed < softSpeed) {
				speed = softSpeed - 0.1;
			}			
			softSpeed=speed;
		}	

		if (speed != 0) {	
			/*
			// Need to pull the arm extension back in while moving the tower arm
			double armExtensionPos=Robot.robotArmExtension.getPos();
			if (pos < 25 && armExtensionPos > RobotMap.armExtendedPickupPos+10) { 
				Robot.robotArmExtension.MoveArmExtension(0.4);
			} else if (pos < 50 && armExtensionPos > RobotMap.armExtendedPlaceLow+10) { 
				Robot.robotArmExtension.MoveArmExtension(0.4);
			} else if (pos > 140 && armExtensionPos > RobotMap.armExtendedPlacePos+10) { 
				Robot.robotArmExtension.MoveArmExtension(0.4);
			}		
			*/

			if (speed < 0) { 
				// Slow down as we approach fully retracted
				if (pos < RobotMap.towerArmRetractedPos + 10 && !Robot.ignoreEncoders) { speed = -.25; }
				if (pos < RobotMap.towerArmRetractedPos && !Robot.ignoreEncoders) { speed = 0; }
			} else if (speed > 0) { 
				// Slow down as we approach fully extended
				if (pos > RobotMap.towerArmExtendedMaxPos - 10 && !Robot.ignoreEncoders) { speed = .25; }
				if (pos > RobotMap.towerArmExtendedMaxPos && !Robot.ignoreEncoders) { speed = 0; }

				if (pos > 20) {
					// Retract the pickup if the arm goes to far out.
					Robot.robotPickup.RetractPickup();
					Robot.robotPickup.cancel();
				}	
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

	/************************************************************************
	 *************************************************************************/

	 public void brakesOn() {
		Robot.TowerArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

    /************************************************************************
	 ************************************************************************/

	 public void brakesOff() {
		Robot.TowerArmMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}	

}