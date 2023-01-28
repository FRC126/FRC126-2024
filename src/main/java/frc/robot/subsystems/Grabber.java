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

		// Do we want brake mode on for the motors?
		//Robot.TowerArmMotorLeft.setNeutralMode(NeutralMode.Brake);
		//Robot.TowerArmMotorRight.setNeutralMode(NeutralMode.Brake);
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

	/************************************************************************
	 ************************************************************************/

	 /************************************************************************
	 * Send power to the drive motors
	 ************************************************************************/

	public void MoveArm(double speed) { 

		if (Robot.internalData.isTeleop()) {
    		// Slow down the turning in teleop
		}

		//SmartDashboard.putNumber("arm speed", speed);

        //TODO Check encoders to if we are at limits.

		Robot.TowerArmMotorLeft.set(speed * RobotMap.TowerArmMotorLeftInversion);
		Robot.TowerArmMotorRight.set(speed * RobotMap.TowerArmMotorRightInversion);
	}

    /************************************************************************
	 ************************************************************************/

	public void resetEncoders() {
		// Need to use encoders for the NEOs
		//Robot.TowerArmMotorLeft.setSelectedSensorPosition(0);
		//Robot.TowerArmMotorRight.setSelectedSensorPosition(0);
	}

}