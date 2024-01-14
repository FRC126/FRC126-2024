/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2024 Code       
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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**********************************************************************************
 **********************************************************************************/

public class SwerveDrive extends SubsystemBase {

	double leftMultiplier, rightMultiplier, leftSpeed, rightSpeed, fbSlowDown, rotSlowDown, limiter;
    double frontLeftRPM, frontRightRPM, rearLeftRPM, rearRightRPM;
	double previousLimiter = 1;
	double fbLast=0;
	double rotLast=0;

	public static SequentialCommandGroup autoCommand;
			
	/************************************************************************
	 ************************************************************************/

	public SwerveDrive() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new SwerveControl(this));

		leftSpeed = 0;
		rightSpeed = 0;

		resetEncoders();
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

    /************************************************************************
	 ************************************************************************/

	public void brakesOn() {
		Robot.swerveFrontRightDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveFrontRightTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveFrontLeftDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveFrontLeftTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveRearLeftDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveRearLeftTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveRearRightDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveRearRightTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	/************************************************************************
	 ************************************************************************/

	public void brakesOff() {
		Robot.swerveFrontRightDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.swerveFrontRightTurnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.swerveFrontLeftDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.swerveFrontLeftTurnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.swerveRearLeftDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.swerveRearLeftTurnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.swerveRearRightDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.swerveRearRightTurnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}

	/************************************************************************
	 ************************************************************************/

	public double getMeanRPM() {
    	frontLeftRPM = Math.abs(Robot.swerveFrontRightDriveRelativeEncoder.getVelocity());
    	frontRightRPM = Math.abs(Robot.swerveFrontLeftDriveRelativeEncoder.getVelocity());
    	rearLeftRPM = Math.abs(Robot.swerveRearLeftDriveRelativeEncoder.getVelocity());
    	rearRightRPM = Math.abs(Robot.swerveRearRightDriveRelativeEncoder.getVelocity());

		SmartDashboard.putNumber("front RPM",(frontLeftRPM+frontRightRPM)/2);
		SmartDashboard.putNumber("rear RPM",(rearLeftRPM+rearRightRPM)/2);

		return((frontLeftRPM + frontRightRPM + rearLeftRPM + rearRightRPM) / 4);
	}

	/************************************************************************
	 ************************************************************************/

	 public void Drive(double y1In, double x1In, double x2In) { 
        Drive(y1In, x1In, x2In, false, 0);
	}		

	/************************************************************************
	 * Send power to the drive motors
	 ************************************************************************/

	public void Drive(double y1In, double x1In, double x2In, boolean driveStraight, double straightDegrees) { 

		double y1 = y1In;
        double x1 = x1In;
		double x2 = x2In;
        double pi = 3.14159;

		SmartDashboard.putNumber("y1In", y1In);
		SmartDashboard.putNumber("x1In", x1In);
        SmartDashboard.putNumber("x2In", x2In);

		Robot.swerveFrontRightDriveMotor.set(y1 * .4);
		Robot.swerveFrontRightTurnMotor.set(x1 * .2);

		Robot.swerveFrontLeftDriveMotor.set(y1 * .4);
		Robot.swerveFrontLeftTurnMotor.set(x1 * .2);

    	Robot.swerveRearLeftDriveMotor.set(y1 * .4);
		Robot.swerveRearLeftTurnMotor.set(x1 * .2);

		Robot.swerveRearRightDriveMotor.set(y1 * .4);
		Robot.swerveRearRightTurnMotor.set(x1 * .2);
	}

    /************************************************************************
	 ************************************************************************/

	public void resetEncoders() {
    	Robot.swerveFrontRightDriveRelativeEncoder.setPosition(0);
    	Robot.swerveFrontLeftDriveRelativeEncoder.setPosition(0);
    	Robot.swerveRearLeftDriveRelativeEncoder.setPosition(0);
    	Robot.swerveRearRightDriveRelativeEncoder.setPosition(0);
	}

    /************************************************************************
	 ************************************************************************/

	public double getDistanceInches() {
		//double wheelDiameter = 6;
		//double gearRatio = 6;
		
		//double left1 = Robot.left1RelativeEncoder.getPosition();
		//double left2 = Robot.left2RelativeEncoder.getPosition();
		//double right1 = Robot.right1RelativeEncoder.getPosition()*-1;
		//double right2 = Robot.right2RelativeEncoder.getPosition()*-1;

		//double avg = Math.abs((left1 + left2 + right1 + right2) / 4);
	
		//double distance = (avg / gearRatio) * (wheelDiameter * 3.1459);

		//SmartDashboard.putNumber("Drive Distance",distance);

		//return(distance);

		return(0);
	}

    /************************************************************************
	 *************************************************************************/

	public void cancel() {
        Drive(0,0,0); 
	}
}

