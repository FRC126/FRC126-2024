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

	static double armRetractedPos=-0.25;
	static double armExtendedPos=1;

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

		double speed=speedIn;

		if (Robot.internalData.isTeleop()) {
    		// TODO
		}

        //TODO Check encoders to if we are at limits.
		double pos=Robot.TowerArmRelativeEncoder.getPosition();

		if ( speed < 0) { 
			if (pos<armRetractedPos) { speed = 0; }
		}

		if ( speed > 0) { 
			if (pos > armExtendedPos) { speed = 0; }
		}

		SmartDashboard.putNumber("Tower Arm Pos", pos);
		SmartDashboard.putNumber("Tower Arm Speed", speed);

		Robot.TowerArmMotor.set(speed * RobotMap.TowerArmMotorInversion);
	}

    /************************************************************************
	 ************************************************************************/

	public void resetEncoders() {
		// Need to use encoders for the NEOs
		Robot.TowerArmRelativeEncoder.setPosition(0);
	}

}