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

public class TowerArm extends SubsystemBase {

	public static double armRetractedPos=-5;
	public static double armPickupPos=0;
	public static double armFloorPickupPos=5;
	public static double armExtendedHighPos=50;
	public static double armExtendedMidPos=100;
	public static double armExtendedLowPos=15;

	/************************************************************************
	 ************************************************************************/

	public TowerArm() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new TowerArmControl(this));

		resetEncoders();

		// Set brake mode for the tower arm motor
		Robot.TowerArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

	/************************************************************************
	 ************************************************************************/

	 /************************************************************************
	 * Send power to the drive motors
	 ************************************************************************/

	public void MoveArm(double speedIn) { 
		double speed = speedIn;

        //Check encoders to if we are at limits.
		double pos = getPos();
		
		SmartDashboard.putBoolean("TA Retracted Limit", Robot.towerArmRetracedLimit.get());
		if ( Robot.towerArmRetracedLimit.get() == false ) {
			// Arm at max extension
		}

		if ( speed < 0) { 
			if (pos<armRetractedPos && !Robot.ignoreEncoders) { speed = 0; }
		}

		if ( speed > 0) { 
			if (pos > armExtendedHighPos && !Robot.ignoreEncoders) { speed = 0; }
		}

		SmartDashboard.putNumber("Tower Arm Pos", pos);
		SmartDashboard.putNumber("Tower Arm Speed", speed);

		Robot.TowerArmMotor.set(speed * RobotMap.TowerArmMotorInversion);

		if (speed == 0) {
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