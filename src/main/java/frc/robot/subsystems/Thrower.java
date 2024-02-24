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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

/**********************************************************************************
 **********************************************************************************/

public class Thrower extends SubsystemBase {	
	static double throwerSpeed[] = { 0,0,0 };
	static int targetReached[] = { 0,0,0 } ;
    static int delay;
    static double P = 0.000025;
    static double I = -0.0003;
	boolean throwerDebug=true;
	public static double myRPM=3000;
    static boolean throwTriggered=false;
	
	// Thrower Angle Control
	PIDController throwerPID;
	boolean reachedAngleTarget=true;
	int reachedAngleCount=0;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Thrower Motors

    TalonFX throwerMotorTalonOne = new TalonFX(RobotMap.throwerTalonMotorOneCanID);
    TalonFX throwerMotorTalonTwo = new TalonFX(RobotMap.throwerTalonMotorTwoCanID);

    CANSparkMax throwerTriggerMotor = new CANSparkMax(RobotMap.throwerTriggerMotorCanID, CANSparkMax.MotorType.kBrushless);
    CANSparkMax throwerClimberMotorLeft = new CANSparkMax(RobotMap.throwerClimberMotorLeftCanID, CANSparkMax.MotorType.kBrushless);
    CANSparkMax throwerClimberMotorRight = new CANSparkMax(RobotMap.throwerClimberMotorRightCanID, CANSparkMax.MotorType.kBrushless);

    RelativeEncoder throwerTriggerMotorRelativeEncoder = throwerTriggerMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, RobotMap.NeoTicksPerRotation);
    RelativeEncoder throwerClimberMotorLeftRelativeEncoder = throwerClimberMotorLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, RobotMap.NeoTicksPerRotation);
    RelativeEncoder throwerClimberMotorRightRelativeEncoder = throwerClimberMotorRight.getEncoder(SparkRelativeEncoder.Type.kHallSensor, RobotMap.NeoTicksPerRotation);	

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Thrower Limit Switches

    DigitalInput throwerBottomLimit = new DigitalInput(8);
    DigitalInput throwerTopLimit = new DigitalInput(7);
    DigitalInput photoSensor = new DigitalInput(2);

	/************************************************************************
	 ************************************************************************/

	public Thrower() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new ThrowerControl(this));
		throwerPID = new PIDController(.1, 0, .0001);
		throwerPID.setTolerance(2,10);
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {
	}

	/************************************************************************
	 ************************************************************************/

	public boolean getPhotoSensor() {
        boolean here=photoSensor.get();
		SmartDashboard.putBoolean("photoSensor",here);
		return(here);
	}
	/************************************************************************
     * Run Main Thower Wheels by target RPM
	 ************************************************************************/

    public int throwerRPM(int index, double targetRPM) {
		double ix, error=0.0, rpm;
		StatusSignal<Double> RPM;

		if (index == 1) {
		    RPM = throwerMotorTalonOne.getVelocity();
 			rpm = RPM.getValueAsDouble() * 60;
		} else {
		    RPM = throwerMotorTalonTwo.getVelocity();
			rpm = RPM.getValueAsDouble() * 60;
		}	

        //String bar="Thrower " + index + " RPM Current foo";
		//SmartDashboard.putNumber(bar,rpm);

		/**********************************************************************
		 * PID Loop for controlling motor RPM
		 **********************************************************************/

		if (targetRPM == 0) { /** Spindown **/
			throwerSpeed[index]=0;
		} else { /** Normal operation **/
			error = targetRPM - rpm;
			ix = error * 0.02; /** Loop frequency **/
			throwerSpeed[index] += P * error + I * ix;
		}

		if(throwerSpeed[index] < 0) {
			throwerSpeed[index] = 0;
		} else if(throwerSpeed[index] > 1) {
			throwerSpeed[index] = 1;
		}

		if (targetRPM < rpm + 125 && targetRPM > rpm - 125) {
			targetReached[index]++;
		} else {
			targetReached[index]=0;
		}

        // Set the speed on the Thrower Motors
		if (index == 1) {
			throwerMotorTalonOne.set(throwerSpeed[index]);
			
			//Robot.throwerMotorTalonOne.set(.5);
		} else {
  			throwerMotorTalonTwo.set(throwerSpeed[index]);
		}	

		if (throwerDebug) {
			// Log info to the smart dashboard
			String foo="Thrower " + index + " RPM Current";
			SmartDashboard.putNumber(foo,rpm);
			foo="Thrower " + index + " RPM Target";
			SmartDashboard.putNumber(foo,targetRPM);
			foo="Thrower " + index + " RPM Reached";
			SmartDashboard.putNumber(foo,targetReached[index]);
			foo="Thrower " + index + " speed";
			SmartDashboard.putNumber(foo,throwerSpeed[index]);
		}

        return(targetReached[index]);
    }

	/************************************************************************
     *push note out for amp
	 ************************************************************************/

    public void setThrowerSpeed(double speed) {
		throwerMotorTalonOne.set(speed);
		throwerMotorTalonTwo.set(speed);
	}

    /************************************************************************
	 ************************************************************************/

	public void resetEncoders() {
        throwerClimberMotorLeftRelativeEncoder.setPosition(-20.0);
		throwerClimberMotorRightRelativeEncoder.setPosition(20.0);
	}

	public double getPosition() {
        double position=0;

		double left = throwerClimberMotorLeftRelativeEncoder.getPosition()*-1;
		double right = throwerClimberMotorRightRelativeEncoder.getPosition();

		position=left+right/2.0;

		return(position);
	}

	public double getThrowerAngle() {
		double position=getPosition();

		double currAngle = (position / RobotMap.NeoTicksPerRotation / 25) * 360;		
		SmartDashboard.putNumber("thrower angle", currAngle);
		SmartDashboard.putNumber("thrower position", position);

		return(currAngle);
	}

    /************************************************************************
	 ************************************************************************/

    public double moveThrower(double speed) {
		double currAngle=getThrowerAngle();
		boolean useLimitSwiches=false;
        
		throwerClimberMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
		throwerClimberMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

		SmartDashboard.putNumber("thrower speed", speed);

		if ( speed < 0 && throwerTopLimit.get() == true && useLimitSwiches) {
			speed=0;
		}		
		if ( speed > 0 && throwerBottomLimit.get() == true && useLimitSwiches ) {
		    speed=0;
			throwerClimberMotorLeftRelativeEncoder.setPosition(50);
			throwerClimberMotorRightRelativeEncoder.setPosition(50);
		}		

		throwerClimberMotorLeft.set(speed*-1);
		throwerClimberMotorRight.set(speed);

        return(currAngle);
	}

    /************************************************************************
	 ************************************************************************/

	public boolean setThrowerPosition(double angle) {

		double currAngle=getThrowerAngle();

		boolean usePID=false;

		if (usePID) {
			if (reachedAngleTarget) {
				reachedAngleTarget=false;
				throwerPID.reset();
			}

			double speed = MathUtil.clamp(throwerPID.calculate(currAngle, angle),-0.1,.1);

			if (throwerPID.atSetpoint()) {
				reachedAngleCount++;
				moveThrower(0);
			} else {
				moveThrower(speed);
				reachedAngleCount=0;
				reachedAngleTarget=false;
			}
		} else {
			if (currAngle < angle - .4) {
			   	    moveThrower(.05);
			    if (currAngle < angle - 3) {
			   	    moveThrower(.25);
				}	
				reachedAngleCount=0;
			} else if (currAngle > angle + .4) {
			   	    moveThrower(-.05);
			    if (currAngle > angle + 3) {
			   	    moveThrower(-.25);
				}	
				reachedAngleCount=0;
			} else {
				reachedAngleCount++;
				moveThrower(0);
			}		
		}	

		if (reachedAngleCount>0) {
			reachedAngleTarget=true;
			return true;			
		} else {
			return false;
		}			

	}

    /************************************************************************
	 ************************************************************************/

	 public void throwerTriggerReverse() {
		throwerTriggerMotor.set(1);
		Robot.pickup.pickupMotorReverse();
	}

    /************************************************************************
	 ************************************************************************/

    public void throwerTriggerOn() {
		throwerTriggerMotor.set(-1);
		Robot.pickup.pickupMotorOn();
		throwTriggered=true;
	}

    /************************************************************************
	 ************************************************************************/

    public void throwerTriggerOff() {
		throwTriggered=false;
		throwerTriggerMotor.set(0);
		Robot.pickup.pickupMotorOff();
	}
	
    /************************************************************************
	 ************************************************************************/

	public void cancel() {
        throwerRPM(1,0); 
        throwerRPM(2,0); 
		throwerTriggerOff();
		moveThrower(0);
	}

    /************************************************************************
	 ************************************************************************/

	public double getRPM() {
		return myRPM;
	}

    /************************************************************************
	 ************************************************************************/

	public void setRPM(double rpmIn) {
		myRPM = rpmIn;
	}

    /************************************************************************
	 ************************************************************************/

	public boolean getThrowTriggered() {
		return throwTriggered;
	}

    /************************************************************************
	 ************************************************************************/

	public void setThrowTriggered(boolean value) {
		throwTriggered = value;
	}
}


