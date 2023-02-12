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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
//import com.revrobotics.SparkMaxAbsoluteEncoder;
//import com.revrobotics.SparkMaxAbsoluteEncoder;
//import com.revrobotics.SparkMaxRelativeEncoder;
//import com.revrobotics.RelativeEncoder;

/**********************************************************************************
 **********************************************************************************/

public class WestCoastDrive extends SubsystemBase {

	double leftMultiplier, rightMultiplier, leftSpeed, rightSpeed, fbSlowDown, rotSlowDown, limiter, left1RPM, left2RPM, right1RPM, right2RPM;
	double previousLimiter = 1;
	double fbLast=0;
    public static CommandBase balanceCommand;
    public static CommandBase climbBalanceCommand;
	public static CommandBase moveLeftCommand;
	public static CommandBase moveRightCommand;
			
	/************************************************************************
	 ************************************************************************/

	public WestCoastDrive() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new DriverControl(this));

		leftSpeed = 0;
		rightSpeed = 0;

		// Do we want brake mode on for the drive motors?
		// brakesOn();
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
	 * Send power to the drive motors
	 ************************************************************************/

	public void Drive(double fbIn, double rotIn) { 

		double fb=fbIn;
		double rot = rotIn;

		SmartDashboard.putNumber("fbIn", fbIn);
        SmartDashboard.putNumber("rotIn", rotIn);

		if (Robot.internalData.isTeleop()) {
    		// Slow down the turning
		    rot = rotIn *.3;
		}

		if (fbIn==0) {
		    fb=0;
			fbLast=0;
		} else if (fbIn > 0) {
			// Soft start for high throttle
			if ( fbIn > fbLast) {
				fb = fbLast+.05;
			} else {
				fb = fbIn;
			} 
			fbLast=fb;
		} else {
			fbLast=0;
			fb=fbIn;
		}
		
		leftMultiplier = fb + (rot);
		rightMultiplier = fb - (rot);

		leftSpeed = leftMultiplier / 1.0;
		rightSpeed = rightMultiplier / 1.0;

		if (leftSpeed > 0) {
			// Handle the difference between forward and backwards in the motors
			rightSpeed = rightSpeed *.95;
		}

		if (rightSpeed < 0) {
			// Handle the difference between forward and backwards in the motors
			leftSpeed = leftSpeed *.95;
		}

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
		SmartDashboard.putBoolean("Drive Limiter", limiter!=0?true:false);

		SmartDashboard.putNumber("drive fb", fb);
		SmartDashboard.putNumber("drive rot", rot);
		
		SmartDashboard.putNumber("Left Speed", leftSpeed);
        SmartDashboard.putNumber("Right Speed", rightSpeed);

		SmartDashboard.putNumber("Left1", Robot.left1RelativeEncoder.getPosition());
		SmartDashboard.putNumber("Left2", Robot.left2RelativeEncoder.getPosition());
		SmartDashboard.putNumber("Right1", Robot.right1RelativeEncoder.getPosition());
		SmartDashboard.putNumber("Right2", Robot.right2RelativeEncoder.getPosition());

		Robot.leftDriveMotor1.set(leftSpeed * RobotMap.left1Inversion);
		Robot.leftDriveMotor2.set(leftSpeed * RobotMap.left2Inversion);

        Robot.rightDriveMotor1.set(rightSpeed * RobotMap.right1Inversion);
		Robot.rightDriveMotor2.set(rightSpeed * RobotMap.right2Inversion);
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
		double ticksPerRotation = 2048;
		double relativeTicksPerRotation=42;
		double wheelDiameter = 6;
		double gearRatio = 3.41;
		
		// Need to use encoders for the NEOs
		double left = Robot.leftDriveEncoder.getAbsolutePosition();
        double right = Robot.rightDriveEncoder.getAbsolutePosition();	

		double left1 = Robot.left1RelativeEncoder.getPosition();
		double left2 = Robot.left2RelativeEncoder.getPosition();
		double right1 = Robot.right1RelativeEncoder.getPosition();
		double right2 = Robot.right2RelativeEncoder.getPosition();

		// Get the absolute value of the average of all the encoders.
		double avg = (left + right) / 2;

		double avg2 = (left1 + left2 + right1 + right2) / 4;

		double distance = ((avg / ticksPerRotation) / gearRatio) * (wheelDiameter * 3.1459);
		
		double distance2 = ((avg2 / relativeTicksPerRotation) / gearRatio) * (wheelDiameter * 3.1459);

		SmartDashboard.putNumber("Drive Distance",distance);
		SmartDashboard.putNumber("Drive 2 Distance",distance2);

		return(distance);
	}

   /************************************************************************
	 ************************************************************************/

	public void doAutoBalance() {
		if (Robot.isAutoBalance) {
			return;
		}	
        Robot.robotArm.MoveArm(0);
        Robot.isAutoBalance = true;
        balanceCommand = new AutoBalance();
		balanceCommand.schedule();
	}

   /************************************************************************
	 ************************************************************************/

    public void stopAutoBalance() {
		if (!Robot.isAutoBalance) {
			return;
		}	
		Robot.isAutoBalance=false;
        balanceCommand.cancel();
		Drive(0,0);
	}

	/************************************************************************
	 ************************************************************************/

	 public void doAutoClimbBalance() {
		if (Robot.isAutoClimbBalance) {
			return;
		}	
        Robot.isAutoClimbBalance = true;
        Robot.robotArm.MoveArm(0);
        climbBalanceCommand = new AutoClimbBalance();
		climbBalanceCommand.schedule();
	}

   /************************************************************************
	 ************************************************************************/

    public void stopAutoClimbBalance() {
		if (!Robot.isAutoClimbBalance) {
			return;
		}	
		Robot.isAutoClimbBalance=false;
        climbBalanceCommand.cancel();
		Drive(0,0);
	}

	/************************************************************************
	 ************************************************************************/

	 public void doAutoMoveLeft() {
		if (Robot.isAutoMoveLeft) {
			return;
		}	
        Robot.isAutoMoveLeft = true;
        Robot.robotArm.MoveArm(0);
        moveLeftCommand = new AutoMoveLeft();
		moveLeftCommand.schedule();
	}

   /************************************************************************
	 ************************************************************************/

    public void stopAutoMoveLeft() {
		if (!Robot.isAutoMoveLeft) {
			return;
		}	
		Robot.isAutoMoveLeft=false;
        moveLeftCommand.cancel();
		Drive(0,0);
	}

	/************************************************************************
	 ************************************************************************/

	 public void doAutoMoveRight() {
		if (Robot.isAutoMoveRight) {
			return;
		}	
        Robot.isAutoMoveRight = true;
        Robot.robotArm.MoveArm(0);
        moveRightCommand = new AutoMoveRight();
		moveRightCommand.schedule();
	}

   /************************************************************************
	 ************************************************************************/

    public void stopAutoMoveRight() {
		if (!Robot.isAutoMoveRight) {
			return;
		}	
		Robot.isAutoMoveRight=false;
        moveRightCommand.cancel();
		Drive(0,0);
	}

}