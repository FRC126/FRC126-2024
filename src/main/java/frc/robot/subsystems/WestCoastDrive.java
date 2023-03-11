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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**********************************************************************************
 **********************************************************************************/

public class WestCoastDrive extends SubsystemBase {

	double leftMultiplier, rightMultiplier, leftSpeed, rightSpeed, fbSlowDown, rotSlowDown, limiter, left1RPM, left2RPM, right1RPM, right2RPM;
	double previousLimiter = 1;
	double fbLast=0;
	double rotLast=0;

	public static SequentialCommandGroup autoCommand;
			
	/************************************************************************
	 ************************************************************************/

	public WestCoastDrive() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new DriverControl(this));

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
		Robot.leftDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.leftDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.rightDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.rightDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

    /************************************************************************
	 ************************************************************************/

	 public void brakesOff() {
		Robot.leftDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.leftDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.rightDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.rightDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}

	/************************************************************************
	 ************************************************************************/

	 public double getMeanRPM() {
		// Need to use encoder for Neo motors
		left1RPM = Math.abs(Robot.left1RelativeEncoder.getVelocity());
		left2RPM = Math.abs(Robot.left2RelativeEncoder.getVelocity());
		right2RPM = Math.abs(Robot.right1RelativeEncoder.getVelocity());
		right1RPM = Math.abs(Robot.right2RelativeEncoder.getVelocity());

		SmartDashboard.putNumber("left RPM",(left1RPM+left2RPM)/2);
		SmartDashboard.putNumber("right RPM",(right1RPM+right2RPM)/2);

		return((left1RPM + left2RPM + right1RPM + right2RPM) / 4);
	}

	/************************************************************************
	 ************************************************************************/

	 public void Drive(double fbIn, double rotIn) { 
        Drive(fbIn, rotIn, false, 0);
	}		

	/************************************************************************
	 * Send power to the drive motors
	 ************************************************************************/

	public void Drive(double fbIn, double rotIn, boolean driveStraight, double straightDegrees) { 

		double fb=fbIn;
		double rot = rotIn;

		//SmartDashboard.putNumber("fbIn", fbIn);
        //SmartDashboard.putNumber("rotIn", rotIn);

		if (Robot.internalData.isTeleop() && !Robot.isAutoCommand ) {
    		// Slow down the turning
		    rot = rotIn *.4;
		}

		// Soft start code
		if (fbIn == 0) {
			// No movement, so fb and last to zero
		    fb = 0;
			fbLast = 0;
		} else if (fbIn > 0) {
			// Soft start for throttle forward
			if ( fbIn > fbLast) {
				fb = fbLast + 0.02;
				if (fbIn > .5) { fb = fbLast + 0.04; }
			}			
			fbLast=fb;
		} else if (fbIn < 0) {
			// Soft start for throttle reverse
			if ( fbIn < fbLast) {
				fb = fbLast - 0.02;
				if (fbIn < -.5) { fb = fbLast - 0.04; }
			}			
			fbLast = fb;
		}

		// Soft turn code
		if (rotIn == 0) {
			// No movement, so rot and last to zero
			rot = 0;
			rotLast = 0;
		} else if (rotIn > 0) {
			// Soft start for throttle forward
			if ( rotIn > rotLast) {
				rot = rotLast + 0.01;
				//if (rotIn > .5) { rot = rotLast + 0.025; }
			}			
			rotLast=rot;
		} else if (rotIn < 0) {
			// Soft start for throttle reverse
			if ( rotIn < rotLast) {
				rot = rotLast - 0.01;
				//if (rotIn < -.5) { rot = rotLast - 0.025; }
			}			
			rotLast = rot;
		}
		
		// If driveStraight is enable, keep the same heading
		if ( driveStraight ) {
			double degrees = Robot.navxMXP.getAngle();
			double tmp = degrees - straightDegrees;

			//SmartDashboard.putNumber("NavX GyroX",degrees);
			//SmartDashboard.putNumber("NavX GyroX Start",straightDegrees);
	
			if ( fb == 0 ) {
				rot = 0;
			} else {
				if(tmp > 0.5) {
					// We are drifiting to the left, correct
					rot = -0.05;
					if (tmp > 3) { rot = -0.1; }
					if (tmp > 5) { rot = -0.2; }
				} else if (tmp < -0.5) {
					// We are drifiting to the right, correct
					rot=0.05;
					if (tmp < -3) { rot = 0.1; }
					if (tmp < -5) { rot = 0.2; }
				} else {
					// Drive straight
					rot = 0;
				}	
			}
		}    			

		// Calculate the speed of the wheels including any turning
		leftMultiplier = fb + (rot);
		rightMultiplier = fb - (rot);
		leftSpeed = leftMultiplier / 1.0;
		rightSpeed = rightMultiplier / 1.0;

		// Don't let the motors brown out the robot
		limiter = 1 + (1 * (Robot.internalData.getVoltage() - Robot.voltageThreshold));
		if (limiter < 0) {
			limiter = 0;
		} else if (limiter > 1) {
			limiter = 1;
		}
		
		previousLimiter = (4 * previousLimiter + limiter) / 5;
		if (Robot.internalData.getVoltage() < Robot.voltageThreshold) {
			leftSpeed *= previousLimiter;
			rightSpeed *= previousLimiter;
		}

		//SmartDashboard.putBoolean("Drive Limiter", limiter!=0?true:false);
		//SmartDashboard.putNumber("drive fb", fb);
		//SmartDashboard.putNumber("drive rot", rot);
		//SmartDashboard.putNumber("Left Speed", leftSpeed);
		//SmartDashboard.putNumber("Right Speed", rightSpeed);
		//SmartDashboard.putNumber("Left1", Robot.left1RelativeEncoder.getPosition());
		//SmartDashboard.putNumber("Left2", Robot.left2RelativeEncoder.getPosition());
		//SmartDashboard.putNumber("Right1", Robot.right1RelativeEncoder.getPosition());
		//SmartDashboard.putNumber("Right2", Robot.right2RelativeEncoder.getPosition());
		
        // Set the Drive Motor Speeds
		Robot.leftDriveMotor1.set(leftSpeed * RobotMap.left1Inversion);
		Robot.leftDriveMotor2.set(leftSpeed * RobotMap.left2Inversion);
        Robot.rightDriveMotor1.set(rightSpeed * RobotMap.right1Inversion);
		Robot.rightDriveMotor2.set(rightSpeed * RobotMap.right2Inversion);

		getDistanceInches();
	}

    /************************************************************************
	 ************************************************************************/

	public void resetEncoders() {
		// Need to use encoders for the NEOs
		Robot.leftDriveEncoder.reset();
        Robot.rightDriveEncoder.reset();

		Robot.left1RelativeEncoder.setPosition(0);
		Robot.left2RelativeEncoder.setPosition(0);
		Robot.right1RelativeEncoder.setPosition(0);
		Robot.right2RelativeEncoder.setPosition(0);
	}

    /************************************************************************
	 ************************************************************************/

	public double getDistanceInches() {
		double wheelDiameter = 6;
		double gearRatio = 6;
		
		double left1 = Robot.left1RelativeEncoder.getPosition();
		double left2 = Robot.left2RelativeEncoder.getPosition();
		double right1 = Robot.right1RelativeEncoder.getPosition()*-1;
		double right2 = Robot.right2RelativeEncoder.getPosition()*-1;

		double avg = Math.abs((left1 + left2 + right1 + right2) / 4);
	
		double distance = (avg / gearRatio) * (wheelDiameter * 3.1459);

		SmartDashboard.putNumber("Drive Distance",distance);

		return(distance);
	}

    /************************************************************************
	 *************************************************************************/

	public void cancel() {
        Drive(0,0); 
	}
}