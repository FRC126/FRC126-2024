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
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.ctre.phoenix6.*;
import com.revrobotics.CANSparkMax;

//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkRelativeEncoder;


/**********************************************************************************
 **********************************************************************************/

public class SwerveDrive extends SubsystemBase {
    boolean swerveDebug=false;

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
		Robot.internalData.resetGyro();
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

	 public double CalcTurnSpeed(double targetAngle, double currentAngle) {
		double speed=0;
		double reverse=1;
		boolean skip=false;

		// If it's quicker, turn the other way
	    if (targetAngle < -0.3 && currentAngle > .3) { reverse=-1; }
        if (targetAngle > .3 && currentAngle < -.3) { reverse=-1; }

		// Skip the fast move if we are just crossing the boundry
        if (targetAngle < -0.45 && currentAngle > .45) { skip=true; }
        if (targetAngle > .45 && currentAngle < -.45) { skip=true; }

     	SmartDashboard.putNumber("reverse angle", reverse);

		if ( targetAngle < (currentAngle) - 0.1 && !skip) {
			speed=-0.4 * reverse;
		} else if (targetAngle > (currentAngle + 0.1) && !skip) {
			speed=0.4 * reverse;
		} else if ( targetAngle < (currentAngle - 0.02) ) {
			speed=-0.1 * reverse;
		} else if (targetAngle > (currentAngle + 0.02) ) {
			speed=0.1 * reverse;
		} else if ( targetAngle < (currentAngle - 0.0010) ) {
			speed=-0.025 * reverse;
		} else if (targetAngle > (currentAngle + 0.0010) ) {
			speed=0.025 * reverse;
		} else if ( targetAngle < (currentAngle - 0.0005) ) {
			speed=-0.01 * reverse;
		} else if (targetAngle > (currentAngle + 0.0005) ) {
			speed=0.01 * reverse;
		}
		return(speed);
	}

	/************************************************************************
	 * Soft start for accelleration to make it more controlable.
	 ************************************************************************/

	 public double smoothWheelSpeed(double input, int index) {
        double result=0;

    	double softStartIncrement=0.02;

		if (driveSlow) {
			// Cap at 20 percent for driveSlow
			if (input > 0.25) { input=0.25; }
		} else {
			// Cap at 50 percent for now
			if (input > 0.45) { input=0.45; }
		}

        if (input > 0) {
			// if the input speed is positive
			if (input <= wheelSpeed[index]) {
				// if the input speed is less than last speed, just set to input
				result = input;
			} else if (input > wheelSpeed[index] ) {
				// if the input speed is greater than the last speed, increment last speed
				// and use that value
				result = wheelSpeed[index] + softStartIncrement;
			}
		} else if (input < 0) {
			// if the input speed is negative
			if (input >= wheelSpeed[index]) {
				// if the input speed is greater than last speed, just set to input
				result = input;
			} else if (input < wheelSpeed[index] ) {
				// if the input speed is less than the last speed, decrement last speed
				// and use that value
				result = wheelSpeed[index] - softStartIncrement;
			}

		}

		// Save the new speed in the class for future smoothing
		wheelSpeed[index] = result;
        return(result);
	}

	/************************************************************************
	 ************************************************************************/

	 public void Drive(double y1In, double x1In, double x2In) { 
        Drive(y1In, x1In, x2In, false, 0);
	}		

	/************************************************************************
	 * Swerve Drive will speed and position calculations
	 * 
	 * https://jacobmisirian.gitbooks.io/frc-swerve-drive-programming/content/chapter1.html
	 ************************************************************************/

	public void Drive(double y1In, double x1In, double x2In, boolean driveStraight, double straightDegrees) { 
		double y1 = y1In;
        double x1 = x1In;
		double x2 = x2In;
		double[] newWheelSpeed = {0,0,0,0};
		double rearRightAngle=0, rearLeftAngle=0, frontRightAngle=0, frontLeftAngle=0;

		if (!swerveDebug) { 
 		    // Log debug data to the smart dashboard
			SmartDashboard.putNumber("y1", y1);
			SmartDashboard.putNumber("x1", x1);
			SmartDashboard.putNumber("x2", x2);
		}	

		// Get the current angle of the robot, and rotate the control inputs the oppsite 
		// direction, and the controls are driver relative, not robot relative
        double currentAngle = Robot.navxMXP.getAngle();

		if (!Robot.isAutoCommand) {
			// 2 dimensional rotation of the control inputs corrected to make the motion
			// driver relative instead of robot relative
			double angle=Math.toRadians(currentAngle); 
			x1 = ( x1In * Math.cos(angle) - (y1In * Math.sin(angle)));
			y1 = ( y1In * Math.cos(angle) + (x1In * Math.sin(angle)));
		}

		if (driveStraight) {
			if (currentAngle < straightDegrees-1.5) {
				x2In=.02;	
			} else if (currentAngle > straightDegrees+1.5) {
				x2In=-.02;	
			} else {
				x2In=0;
			}
		}

		// Get the Encoder information from each swerve drive module
    	StatusSignal FRPosSS = Robot.SwerveFrontRightEncoder.getAbsolutePosition();
		StatusSignal FLPosSS = Robot.SwerveFrontLeftEncoder.getAbsolutePosition();
		StatusSignal RRPosSS = Robot.SwerveRearRightEncoder.getAbsolutePosition();
		StatusSignal RLPosSS = Robot.SwerveRearLeftEncoder.getAbsolutePosition();

		double frontRightPos = FRPosSS.getValueAsDouble();
		double frontLeftPos = FLPosSS.getValueAsDouble();
		double rearRightPos = RRPosSS.getValueAsDouble();
		double rearLeftPos = RLPosSS.getValueAsDouble();

		if (y1 == 0 && x1 == 0 && x2 == 0) {
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

			rearRightAngle = Math.atan2 (a, d) / pi *.49;
			rearLeftAngle = Math.atan2 (a, c) / pi * .49;
			frontRightAngle = Math.atan2 (b, d) / pi * .49;
			frontLeftAngle = Math.atan2 (b, c) / pi * .49;

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
 		    // Log debug data to the smart dashboard
			SmartDashboard.putNumber("y1 Rotate", y1);
			SmartDashboard.putNumber("x1 Rotate", x1);

     		SmartDashboard.putNumber("currentAngle", currentAngle);

			SmartDashboard.putNumber("frontRightPos", frontRightPos);
			SmartDashboard.putNumber("frontLeftPos", frontLeftPos);
			SmartDashboard.putNumber("rearRightPos", rearRightPos);
			SmartDashboard.putNumber("rearLeftPos", rearLeftPos);

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
		double gearRatio = SmartDashboard.getNumber(Robot.GEAR_RATIO, 8.14);
		// double gearRatio = 8.14;
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

///////////////////////////////////////////////////////////////////////////////////////
// Controls
//
// Left Joystick - Driver Relative Robot Movement
//
// Right Joystick - Robot Rotation
//
// Left Trigger - Slow Mode
// Right Trigger - Brake Mode
//
// B Button - Reset Gyro to 0, do it when front of robot is facing directly away from the driver
//