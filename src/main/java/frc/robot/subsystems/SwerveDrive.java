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

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.*;

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
    double pi = 3.14159;
    double inchesPerMeter = 2.54/100;

	public static SequentialCommandGroup autoCommand;
	public final double LENGTH = 26 * inchesPerMeter;
	public final double WIDTH = 26 * inchesPerMeter;
			
	/************************************************************************
	 ************************************************************************/

	public SwerveDrive() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new SwerveControl(this));

		leftSpeed = 0;
		rightSpeed = 0;

		Robot.swerveFrontRightTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveFrontLeftTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveRearLeftTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveRearRightTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

    /************************************************************************
	 ************************************************************************/

	public void brakesOn() {
		Robot.swerveFrontRightDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveFrontLeftDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveRearLeftDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveRearRightDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		Robot.swerveFrontRightTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveFrontLeftTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveRearLeftTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.swerveRearRightTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	/************************************************************************
	 ************************************************************************/

	public void brakesOff() {
		Robot.swerveFrontRightDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.swerveFrontLeftDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.swerveRearLeftDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.swerveRearRightDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

		Robot.swerveFrontRightTurnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.swerveFrontLeftTurnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.swerveRearLeftTurnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
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
	 ************************************************************************/

	 public double CalcTurnSpeed(double targetAngle, double currentAngle) {
		    double speed=0;

            if ( targetAngle < (currentAngle) - 0.1 ) {
				speed=-0.25;
			} else if (targetAngle > (currentAngle + 0.1) ) {
				speed=0.25;
            } else if ( targetAngle < (currentAngle - 0.01) ) {
				speed=-0.1;
			} else if (targetAngle > (currentAngle + 0.01) ) {
				speed=.1;
            } else if ( targetAngle < (currentAngle - 0.0015) ) {
				speed=-0.025;
			} else if (targetAngle > (currentAngle + 0.0015) ) {
				speed=0.025;
			}
			return(speed);
	}

	/************************************************************************
	 * Send power to the drive motors
	 ************************************************************************/

	public void Drive(double y1In, double x1In, double x2In, boolean driveStraight, double straightDegrees) { 

		double y1 = y1In;
        double x1 = x1In;
		double x2 = x2In;
		double rearRightSpeed, rearLeftSpeed, frontRightSpeed, frontLeftSpeed;
		double rearRightAngle, rearLeftAngle, frontRightAngle, frontLeftAngle;

		if (y1 == 0 && x1== 0 && x2 == 0) {
			// If not joysticks are moved, just stop the motors and return
            Robot.swerveFrontRightDriveMotor.set(0);
			Robot.swerveFrontLeftDriveMotor.set(0);
			Robot.swerveRearLeftDriveMotor.set(0);
			Robot.swerveRearRightDriveMotor.set(0);
   			Robot.swerveFrontRightTurnMotor.set(0);
   			Robot.swerveFrontLeftTurnMotor.set(0);
   			Robot.swerveRearRightTurnMotor.set(0);
   			Robot.swerveRearLeftTurnMotor.set(0);
            return;
		}

   		SmartDashboard.putNumber("y1In", y1In);
		SmartDashboard.putNumber("x1In", x1In);
		SmartDashboard.putNumber("x2In", x2In);

		if (true) {
			double r = Math.sqrt ((LENGTH * LENGTH) + (WIDTH * WIDTH));
			y1 *= -1;
		
			double a = x1 - x2 * (LENGTH / r);
			double b = x1 + x2 * (LENGTH / r);
			double c = y1 - x2 * (WIDTH / r);
			double d = y1 + x2 * (WIDTH / r);
		
			rearRightSpeed = Math.sqrt ((a * a) + (d * d));
			rearLeftSpeed = Math.sqrt ((a * a) + (c * c));
			frontRightSpeed = Math.sqrt ((b * b) + (d * d));
			frontLeftSpeed = Math.sqrt ((b * b) + (c * c));
			
			rearRightAngle = Math.atan2 (a, d) / pi *.48;
			rearLeftAngle = Math.atan2 (a, c) / pi * .48;
			frontRightAngle = Math.atan2 (b, d) / pi * .48;
			frontLeftAngle = Math.atan2 (b, c) / pi * .48;

	   		SmartDashboard.putNumber("backRightSpeed", rearRightSpeed);
			SmartDashboard.putNumber("backLeftSpeed", rearLeftSpeed);
			SmartDashboard.putNumber("frontRightSpeed", frontRightSpeed);
			SmartDashboard.putNumber("frontLeftSpeed", frontLeftSpeed);

			SmartDashboard.putNumber("backRightAngle", rearRightAngle);
			SmartDashboard.putNumber("backLeftAngle", rearLeftAngle);
			SmartDashboard.putNumber("frontRightAngle", frontRightAngle);
			SmartDashboard.putNumber("frontLeftAngle", frontLeftAngle);

			StatusSignal FRPosSS = Robot.SwerveFrontRightEncoder.getAbsolutePosition();
			StatusSignal FLPosSS = Robot.SwerveFrontLeftEncoder.getAbsolutePosition();
			StatusSignal RRPosSS = Robot.SwerveRearRightEncoder.getAbsolutePosition();
			StatusSignal RLPosSS = Robot.SwerveRearLeftEncoder.getAbsolutePosition();

			double frontRightPos = FRPosSS.getValueAsDouble();
			double frontLeftPos = FLPosSS.getValueAsDouble();
			double rearRightPos = RRPosSS.getValueAsDouble();
			double rearLeftPos = RLPosSS.getValueAsDouble();

    		SmartDashboard.putNumber("frontRightPos", frontRightPos);
			SmartDashboard.putNumber("frontLeftPos", frontLeftPos);
			SmartDashboard.putNumber("rearRightPos", rearRightPos);
			SmartDashboard.putNumber("rearLeftPos", rearLeftPos);

			Robot.swerveFrontRightDriveMotor.set(frontRightSpeed * .4);
			Robot.swerveFrontLeftDriveMotor.set(rearLeftSpeed * .4 *-1);
			Robot.swerveRearLeftDriveMotor.set(rearLeftSpeed * .4 * -1);
			Robot.swerveRearRightDriveMotor.set(rearRightSpeed * .4);

			double speed; 
			speed=CalcTurnSpeed(frontRightPos,frontRightAngle);
   			Robot.swerveFrontRightTurnMotor.set(speed);

			speed=CalcTurnSpeed(frontLeftPos,frontLeftAngle);
   			Robot.swerveFrontLeftTurnMotor.set(speed);

			speed=CalcTurnSpeed(rearRightPos,rearRightAngle);
   			Robot.swerveRearRightTurnMotor.set(speed);

			speed=CalcTurnSpeed(rearLeftPos,rearLeftAngle);
   			Robot.swerveRearLeftTurnMotor.set(speed);
		}
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

