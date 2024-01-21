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
    boolean swerveDebug=true;

	double[] wheelSpeed = {0,0,0,0};

	static final int frontLeft=0;
	static final int frontRight=1;
	static final int rearLeft=2;
	static final int rearRight=3;

	boolean areBrakesOn = false;
	boolean driveSlow = false;
	
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

		wheelSpeed[frontLeft] = 0;
		wheelSpeed[frontRight] = 0;
		wheelSpeed[rearLeft] = 0;
		wheelSpeed[rearRight] = 0;

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

	public void resetYaw() {
		Robot.navxMXP.zeroYaw();
	} 

    /************************************************************************
	 ************************************************************************/

	public void driveSlow(boolean in) {
		driveSlow=in;
	} 

    /************************************************************************
	 ************************************************************************/

	public void brakesOn() {
		if (!areBrakesOn) {
			Robot.swerveFrontRightDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
			Robot.swerveFrontLeftDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
			Robot.swerveRearLeftDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
			Robot.swerveRearRightDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
			areBrakesOn=true;
		}	
	}

	/************************************************************************
	 ************************************************************************/

	public void brakesOff() {
		if (areBrakesOn) {
			Robot.swerveFrontRightDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
			Robot.swerveFrontLeftDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
			Robot.swerveRearLeftDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
			Robot.swerveRearRightDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
			areBrakesOn=false;
		}	
	}

	/************************************************************************
	 ************************************************************************/

	public double getMeanRPM() {
    	frontLeftRPM = Math.abs(Robot.swerveFrontRightDriveRelativeEncoder.getVelocity());
    	frontRightRPM = Math.abs(Robot.swerveFrontLeftDriveRelativeEncoder.getVelocity());
    	rearLeftRPM = Math.abs(Robot.swerveRearLeftDriveRelativeEncoder.getVelocity());
    	rearRightRPM = Math.abs(Robot.swerveRearRightDriveRelativeEncoder.getVelocity());

		if (swerveDebug) { 
			SmartDashboard.putNumber("front RPM",(frontLeftRPM+frontRightRPM)/2);
			SmartDashboard.putNumber("rear RPM",(rearLeftRPM+rearRightRPM)/2);
		}		

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
				speed=-0.4;
			} else if (targetAngle > (currentAngle + 0.1) ) {
				speed=0.4;
            } else if ( targetAngle < (currentAngle - 0.01) ) {
				speed=-0.1;
			} else if (targetAngle > (currentAngle + 0.01) ) {
				speed=0.1;
            } else if ( targetAngle < (currentAngle - 0.0010) ) {
				speed=-0.02;
			} else if (targetAngle > (currentAngle + 0.0010) ) {
				speed=0.02;
			}
			return(speed);
	}

	/************************************************************************
	 * Send power to the drive motors
	 ************************************************************************/

	 public double smoothWheelSpeed(double input, int index) {
        double result=0;

		if (driveSlow) {
			if (input > 0.2) { 
				// Cap at 20 percent for driveSlow
				input=0.2;
			}
		} else {
			if (input > 0.4) { 
				// Cap at 40 percent for now
				input=0.4;
			}
		}

        if (input > 0) {
			// if the input speed is positive
			if (input <= wheelSpeed[index]) {
				// if the input speed is less than last speed, just set to input
				result=input;
			} else if (input > wheelSpeed[index] ) {
				// if the input speed is greater than the last speed, increment last speed
				// and use that value
				result = wheelSpeed[index]+0.02;
			}
		} else if (input > 0) {
			// if the input speed is negative
			if (input >= wheelSpeed[index]) {
				// if the input speed is greater than last speed, just set to input
				result=input;
			} else if (input < wheelSpeed[index] ) {
				// if the input speed is less than the last speed, decrement last speed
				// and use that value
				result = wheelSpeed[index]-0.02;
			}

		}

		wheelSpeed[index] = result;
        return(result);
	}
	
	/************************************************************************
	 * Send power to the drive motors
	 ************************************************************************/

	public void Drive(double y1In, double x1In, double x2In, boolean driveStraight, double straightDegrees) { 

		double y1 = y1In;
        double x1 = x1In;
		double x2 = x2In;
		double[] newWheelSpeed = {0,0,0,0};
		double rearRightAngle=0, rearLeftAngle=0, frontRightAngle=0, frontLeftAngle=0;

		// Get the current angle of the robot, and rotate the control inputs the oppsite 
		// direction, and the controls are driver relative, not robot relative
        double currentAngle = Robot.navxMXP.getAngle();
		// 2 dimensional rotation of the control inputs corrected to make the motion
		// driver relative instead of robot relative
		x1 = ( x1In * Math.cos(currentAngle*-1) - (y1In * Math.sin(currentAngle*-1)));
        y1 = ( y1In * Math.cos(currentAngle*-1) + (x1In * Math.sin(currentAngle*-1)));

		// Get the Encoder information from each swerve drive module
    	StatusSignal FRPosSS = Robot.SwerveFrontRightEncoder.getAbsolutePosition();
		StatusSignal FLPosSS = Robot.SwerveFrontLeftEncoder.getAbsolutePosition();
		StatusSignal RRPosSS = Robot.SwerveRearRightEncoder.getAbsolutePosition();
		StatusSignal RLPosSS = Robot.SwerveRearLeftEncoder.getAbsolutePosition();

		double frontRightPos = FRPosSS.getValueAsDouble();
		double frontLeftPos = FLPosSS.getValueAsDouble();
		double rearRightPos = RRPosSS.getValueAsDouble();
		double rearLeftPos = RLPosSS.getValueAsDouble();

		if (swerveDebug) { 
 		    // Log debug data to the smart dashboard
     		SmartDashboard.putNumber("currentAngle", currentAngle);

			SmartDashboard.putNumber("frontRightPos", frontRightPos);
			SmartDashboard.putNumber("frontLeftPos", frontLeftPos);
			SmartDashboard.putNumber("rearRightPos", rearRightPos);
			SmartDashboard.putNumber("rearLeftPos", rearLeftPos);

			SmartDashboard.putNumber("y1In", y1In);
			SmartDashboard.putNumber("x1In", x1In);
			SmartDashboard.putNumber("x2In", x2In);
		}	

		if (y1 == 0 && x1== 0 && x2 == 0) {
			// If not joysticks are moved, just stop the motors, and
			// zero the speed offsets
            Robot.swerveFrontRightDriveMotor.set(0);
			Robot.swerveFrontLeftDriveMotor.set(0);
			Robot.swerveRearLeftDriveMotor.set(0);
			Robot.swerveRearRightDriveMotor.set(0);

			Robot.swerveFrontRightTurnMotor.set(0);
   			Robot.swerveFrontLeftTurnMotor.set(0);
   			Robot.swerveRearRightTurnMotor.set(0);
   			Robot.swerveRearLeftTurnMotor.set(0);

	        newWheelSpeed[frontRight] = smoothWheelSpeed(0,frontRight);
			newWheelSpeed[frontLeft] = smoothWheelSpeed(0,frontLeft);
			newWheelSpeed[rearRight] = smoothWheelSpeed(0,rearRight);
			newWheelSpeed[rearLeft] = smoothWheelSpeed(0,rearLeft);
		} else {
			// Calculate the speed and position of the 4 wheels based on 
			// the joystick input

			double r = Math.sqrt ((LENGTH * LENGTH) + (WIDTH * WIDTH));
			y1 *= -1;
		
			double a = x1 - x2 * (LENGTH / r);
			double b = x1 + x2 * (LENGTH / r);
			double c = y1 - x2 * (WIDTH / r);
			double d = y1 + x2 * (WIDTH / r);
		
			newWheelSpeed[rearRight] = Math.sqrt ((a * a) + (d * d));
			newWheelSpeed[rearLeft] = Math.sqrt ((a * a) + (c * c));
			newWheelSpeed[frontRight] = Math.sqrt ((b * b) + (d * d));
			newWheelSpeed[frontLeft] = Math.sqrt ((b * b) + (c * c));

			rearRightAngle = Math.atan2 (a, d) / pi *.48;
			rearLeftAngle = Math.atan2 (a, c) / pi * .48;
			frontRightAngle = Math.atan2 (b, d) / pi * .48;
			frontLeftAngle = Math.atan2 (b, c) / pi * .48;

			// Run the turning motors based on the calculated target
			Robot.swerveFrontRightTurnMotor.set(CalcTurnSpeed(frontRightPos,frontRightAngle));
			Robot.swerveFrontLeftTurnMotor.set(CalcTurnSpeed(frontLeftPos,frontLeftAngle));
			Robot.swerveRearRightTurnMotor.set(CalcTurnSpeed(rearRightPos,rearRightAngle));
			Robot.swerveRearLeftTurnMotor.set(CalcTurnSpeed(rearLeftPos,rearLeftAngle));
			
			// Smooth the wheel speed so the robot isn't so jumpy
			newWheelSpeed[frontRight] = smoothWheelSpeed(newWheelSpeed[frontRight],frontRight);
			newWheelSpeed[frontLeft] = smoothWheelSpeed(newWheelSpeed[frontLeft],frontLeft);
			newWheelSpeed[rearRight] = smoothWheelSpeed(newWheelSpeed[rearRight],rearRight);
			newWheelSpeed[rearLeft] = smoothWheelSpeed(newWheelSpeed[rearLeft],rearLeft);

			// Run the drive motors to the smoothed speed
			Robot.swerveFrontRightDriveMotor.set(newWheelSpeed[frontRight]);
			Robot.swerveFrontLeftDriveMotor.set(newWheelSpeed[frontLeft] * -1);
			Robot.swerveRearLeftDriveMotor.set(newWheelSpeed[rearLeft] * -1);
			Robot.swerveRearRightDriveMotor.set(newWheelSpeed[rearRight]);
		}

		if (swerveDebug) { 
			// Debug data to the smart dashboard.
			SmartDashboard.putNumber("frontRightSpeed", newWheelSpeed[frontRight]);
			SmartDashboard.putNumber("frontLeftSpeed", newWheelSpeed[frontLeft]);
			SmartDashboard.putNumber("rearRightSpeed", newWheelSpeed[rearRight]);
			SmartDashboard.putNumber("rearLeftSpeed", newWheelSpeed[rearLeft]);

			SmartDashboard.putNumber("frontRightAngle", frontRightAngle);
			SmartDashboard.putNumber("frontLeftAngle", frontLeftAngle);
			SmartDashboard.putNumber("rearRightAngle", rearRightAngle);
			SmartDashboard.putNumber("rearLeftAngle", rearLeftAngle);
  		}
	}

    /************************************************************************
	 ************************************************************************/

	public void resetEncoders() {
    	Robot.swerveFrontLeftDriveRelativeEncoder.setPosition(0);
    	Robot.swerveRearLeftDriveRelativeEncoder.setPosition(0);
    	Robot.swerveFrontRightDriveRelativeEncoder.setPosition(0);
    	Robot.swerveRearRightDriveRelativeEncoder.setPosition(0);
	}

    /************************************************************************
	 ************************************************************************/

	public double getDistanceInches() {
		double wheelDiameter = 4;
		// L1 ratio
		double gearRatio = 8.14;
		// L2 ratio
		// double gearRatio = 6.75;
		// L3 ratio
		// double gearRatio = 6.12;
		
		double left1 = Robot.swerveFrontLeftDriveRelativeEncoder.getPosition() * -1;
		double left2 = Robot.swerveFrontLeftDriveRelativeEncoder.getPosition() * -1;
		double right1 = Robot.swerveFrontRightDriveRelativeEncoder.getPosition();
		double right2 = Robot.swerveRearRightDriveRelativeEncoder.getPosition();

		double avg = Math.abs((left1 + left2 + right1 + right2) / 4);
	
		double distance = (avg / gearRatio) * (wheelDiameter * 3.1459);

		if (swerveDebug) { 
  			SmartDashboard.putNumber("Drive Distance",distance);
		}	

		return(distance);
	}

    /************************************************************************
	 *************************************************************************/

	public void cancel() {
        Drive(0,0,0); 
	}
}

