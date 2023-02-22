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

	public static double armRetractedPos=0;
	public static double armExtendedPlacePos=340;
	public static double armExtendedPickupPos=40;
	public static double armExtendedPickupFloorPos=90;
	public static double armExtendedPlaceLow=150;
	//public static double armExtendedMaxPos=460;
	public static double armExtendedMaxPos=385;
	double lastSpeed=1000;

	/************************************************************************
	 ************************************************************************/

	public ArmExtension() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new ArmExtensionControl(this));

		//resetEncoders();

		// Brake mode on for the motor
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

	public void MoveArmExtension(double speedIn) { 
		double speed = speedIn;

		//  Check encoders to if we are at limits.
		double pos = getPos();

		SmartDashboard.putNumber("Arm Extension Pos", pos);

		//if (speed != 0) {
			
			//SmartDashboard.putBoolean("AE Bottom Limit", Robot.armExtensionBottomLimit.get());
			//if ( Robot.armExtensionBottomLimit.get() == false ) {
			//	// Arm at max extension
			//	speed = 0;
			//}

			//SmartDashboard.putBoolean("AE Top Limit", Robot.armExtensionTopLimit.get());
			//if ( Robot.armExtensionTopLimit.get() == false ) {
			//	// Arm in the minimum extension
			//	speed = 0;
			//}

			if (speed > 0) { 
				if (pos < armRetractedPos + 35 && !Robot.ignoreEncoders) { speed = 0.2; }
				if (pos < armRetractedPos + 15 && !Robot.ignoreEncoders) { speed = 0.1; }
				if (pos < armRetractedPos && !Robot.ignoreEncoders) { speed = 0; }
			}

			if (speed < 0) { 
				if (pos > armExtendedMaxPos - 35 && !Robot.ignoreEncoders) { speed = -.2; }
				if (pos > armExtendedMaxPos - 15 && !Robot.ignoreEncoders) { speed = -.1; }
				if (pos > armExtendedMaxPos && !Robot.ignoreEncoders) { speed = 0; }
			}

			SmartDashboard.putNumber("Arm Extension Speed", speedIn);
			SmartDashboard.putNumber("Arm Extension Speed", speed);

		//}

		if (lastSpeed != speed) {
		    Robot.ArmExtensionMotor.set(speed * RobotMap.ArmExtensionMotorInversion);
			lastSpeed = speed;
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

}