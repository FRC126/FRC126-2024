/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2022 Code       
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

public class WestCoastDrive extends SubsystemBase {

	double leftMultiplier, rightMultiplier, leftSpeed, rightSpeed, fbSlowDown, rotSlowDown, limiter, left1RPM, left2RPM, right1RPM, right2RPM;
	double previousLimiter = 1;
		
	/************************************************************************
	 ************************************************************************/

	public WestCoastDrive() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new DriverControl(this));

		leftSpeed = 0;
		rightSpeed = 0;

		// Do we want brake mode on for the drive motors?
		//Robot.leftDriveMotor1.setNeutralMode(NeutralMode.Brake);
		//Robot.leftDriveMotor2.setNeutralMode(NeutralMode.Brake);
		//Robot.rightDriveMotor1.setNeutralMode(NeutralMode.Brake);
		//Robot.rightDriveMotor2.setNeutralMode(NeutralMode.Brake);

	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

	/************************************************************************
	 ************************************************************************/

	 public double getMeanRPM() {
		// Need to use encoder for Neo motors
		//left1RPM = Math.abs(Robot.leftDriveMotor1.getSelectedSensorVelocity() / 3.41);
		//left2RPM = Math.abs(Robot.leftDriveMotor2.getSelectedSensorVelocity() / 3.41);
		//right1RPM = Math.abs(Robot.rightDriveMotor1.getSelectedSensorVelocity() / 3.41);
		//right2RPM = Math.abs(Robot.rightDriveMotor2.getSelectedSensorVelocity() / 3.41);
		left1RPM=left2RPM=right1RPM=right2RPM=0;
		return((left1RPM + left2RPM + right1RPM + right2RPM) / 4);
	}

	/************************************************************************
	 * Send power to the drive motors
	 ************************************************************************/

	public void Drive(double fb, double rot_in) { 

		double rot = rot_in;
		if (Robot.internalData.isTeleop()) {
    		// Slow down the turning
		    rot = rot_in *.6;
		}

		leftMultiplier = fb + (rot);
		rightMultiplier = fb - (rot);
		leftSpeed = leftMultiplier / 1.0;
		rightSpeed = rightMultiplier / 1.0;

		limiter = 1 + (1 * (Robot.internalData.getVoltage() - Robot.voltageThreshold));
		if(limiter < 0) {
			limiter = 0;
		} else if(limiter > 1) {
			limiter = 1;
		}
		previousLimiter = (4 * previousLimiter + limiter) / 5;
		if(Robot.internalData.getVoltage() < Robot.voltageThreshold) {
			leftSpeed *= previousLimiter;
			rightSpeed *= previousLimiter;
		}

		//SmartDashboard.putNumber("drive fb", fb);
		//SmartDashboard.putNumber("drive rot", rot);
		//SmartDashboard.putNumber("Left Speed", leftSpeed);
        //SmartDashboard.putNumber("Right Speed", rightSpeed);

		Robot.leftDriveMotor1.set(leftSpeed * RobotMap.left1Inversion);
		//Robot.leftDriveMotor2.set(leftSpeed * RobotMap.left2Inversion);

        Robot.rightDriveMotor1.set(rightSpeed * RobotMap.right1Inversion);
		//Robot.rightDriveMotor2.set(rightSpeed * RobotMap.right2Inversion);
	}

    /************************************************************************
	 ************************************************************************/

	public void resetEncoders() {
		// Need to use encoders for the NEOs
		//Robot.leftDriveMotor1.setSelectedSensorPosition(0);
		//Robot.leftDriveMotor2.setSelectedSensorPosition(0);

        //Robot.rightDriveMotor1.setSelectedSensorPosition(0);
		//Robot.rightDriveMotor2.setSelectedSensorPosition(0);
	}

    /************************************************************************
	 ************************************************************************/

	public double getDistanceInches() {
		double ticksPerRotation=2048;
		double wheelDiameter = 6.45; // 6.4 inches, 20.25" diameter
		double gearRatio = 3.41;
		
		// Need to use encoders for the NEOs
		// double left1 = Math.abs(Robot.leftDriveMotor1.getSelectedSensorPosition() * RobotMap.left1Inversion);
		// double left2 = Math.abs(Robot.leftDriveMotor2.getSelectedSensorPosition() * RobotMap.left2Inversion);

        // double right1 = Math.abs(Robot.rightDriveMotor1.getSelectedSensorPosition() * RobotMap.right1Inversion);
		// double right2 = Math.abs(Robot.rightDriveMotor2.getSelectedSensorPosition() * RobotMap.right2Inversion);

		double left1 = 0;
		double left2 = 0;
        double right1 = 0;
		double right2 = 0;

		// Get the absolute value of the average of all the encoders.
		double avg = (left1 + left2 + right1+ right2) / 4;

		double distance = ((avg / ticksPerRotation) / gearRatio) * (wheelDiameter *3.1459);

		SmartDashboard.putNumber("Drive Distance",distance);

		return(distance);
	}
}