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

public class ArmExtension extends SubsystemBase {
	double lastSpeed=1000;
	int limitHit=0;
	double softSpeed=0;

	/************************************************************************
	 ************************************************************************/

	public ArmExtension() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new ArmExtensionControl(this));

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

	public void MoveArmExtension(double speedIn) { 
		double speed = speedIn;

		//  Check encoders to if we are at limits.
		double pos = getPos();

		SmartDashboard.putNumber("Arm Extension Pos", pos);

		if (Robot.armExtensionBottomLimit.get() == false) {
			SmartDashboard.putBoolean("Arm Extension Limit", true);
			limitHit=0;
		} else {
		    SmartDashboard.putBoolean("Arm Extension Limit", false);
			if (speed > 0) { speed=0; }
			limitHit++;
			if (limitHit > 20) {
  			     Robot.ArmExtensionRelativeEncoder.setPosition(5);
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
			double towerArmPos=Robot.robotTowerArm.getPos();

			if (speed == 0) {
				// Need to pull the arm extension back in while moving the tower arm
				if (towerArmPos < 25 && pos > RobotMap.armExtendedPickupPos+10) { speed=0.4; }
				if (towerArmPos < 50 && pos > RobotMap.armExtendedPlaceLow+10) { speed=0.4; }
				if (towerArmPos > 140 && pos > RobotMap.armExtendedPlacePos+10) { speed=0.4; }		
			}
			
			if (speed > 0) { 
				if (pos < RobotMap.armRetractedPos + 30 && !Robot.ignoreEncoders) { speed = 0.4; }
				if (pos < RobotMap.armRetractedPos + 15 && !Robot.ignoreEncoders) { speed = 0.2; }
				if (pos < RobotMap.armRetractedPos && !Robot.ignoreEncoders) { speed = 0; }
			} else if (speed < 0) { 
				if (pos > RobotMap.armExtendedMaxPos - 30 && !Robot.ignoreEncoders) { speed = -.4; }
				if (pos > RobotMap.armExtendedMaxPos - 15 && !Robot.ignoreEncoders) { speed = -.2; }
				if (pos > RobotMap.armExtendedMaxPos && !Robot.ignoreEncoders) { speed = 0; }
                
				if (towerArmPos < 25 && pos > RobotMap.armExtendedPickupPos+70) { speed=0; }
				if (towerArmPos < 50 && pos > RobotMap.armExtendedPlaceLow+40) { speed=0; }
				if (towerArmPos > 140 && pos > RobotMap.armExtendedPlacePos+10) { speed=0; }		
			}
        }

		if (lastSpeed != speed) {
		    Robot.ArmExtensionMotor.set(speed * RobotMap.ArmExtensionMotorInversion);
			lastSpeed = speed;
			//SmartDashboard.putNumber("Arm Extension Speed", speed);
		}	

		if (speed == 0 && lastSpeed != 0) {
			Robot.ArmExtensionMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		}
	}

    /************************************************************************
	 ************************************************************************/

	public void resetEncoders() {
		// Need to use encoders for the NEOs
		Robot.ArmExtensionRelativeEncoder.setPosition(0);
	}

    /************************************************************************
	 ************************************************************************/

	 public double getPos() {
		// Need to use encoders for the NEOs
		return(Robot.ArmExtensionRelativeEncoder.getPosition() * -1);
	}

	/************************************************************************
	 *************************************************************************/

	 public void cancel() {
        MoveArmExtension(0); 
	}

	/************************************************************************
	 *************************************************************************/

	public void brakesOn() {
		Robot.ArmExtensionMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

    /************************************************************************
	 ************************************************************************/

	 public void brakesOff() {
		Robot.ArmExtensionMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}	
}