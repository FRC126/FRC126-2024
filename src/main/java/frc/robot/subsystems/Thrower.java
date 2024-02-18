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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix6.*;
import com.revrobotics.CANSparkMax;

/**********************************************************************************
 **********************************************************************************/

public class Thrower extends SubsystemBase {	
	static double throwerSpeed[] = { 0,0,0 };
	static int targetReached[] = { 0,0,0 } ;
    static int delay;
    static double P = 0.000025;
    static double I = -0.0003;
	boolean throwerDebug=true;
	public static double myRPM=1500;
	
	// Thrower Angle Control
	PIDController throwerPID;
	boolean reachedAngleTarget=true;
	int reachedAngleCount=0;

	/************************************************************************
	 ************************************************************************/

	public Thrower() {

		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new ThrowerControl(this));
		//throwerPID = new PIDController(.1, 0, .0001);
		//throwerPID.setTolerance(2,10);
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {
	}

	/************************************************************************
	 ************************************************************************/

	public boolean getPhotoSensor() {
        boolean here=Robot.photoSensor.get();
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
		    RPM = Robot.throwerMotorTalonOne.getVelocity();
 			rpm = RPM.getValueAsDouble() * 60;
		} else {
		    RPM = Robot.throwerMotorTalonTwo.getVelocity();
			rpm = RPM.getValueAsDouble() * 60;
		}	
        String bar="Thrower " + index + " RPM Current foo";
			SmartDashboard.putNumber(bar,rpm);
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
			Robot.throwerMotorTalonOne.set(throwerSpeed[index]);
			
			//Robot.throwerMotorTalonOne.set(.5);
		} else {
  			Robot.throwerMotorTalonTwo.set(throwerSpeed[index]);
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
	 ************************************************************************/

    public double moveThrower(double speed) {
        double position=0;

		Robot.throwerClimberMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.throwerClimberMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

		double left = Robot.throwerClimberMotorLeftRelativeEncoder.getPosition();
		double right = Robot.throwerClimberMotorRightRelativeEncoder.getPosition()*-1;

		position=left+right/2.0;

		if ( speed < 0 && Robot.throwerTopLimit.get() == true ) {
		//	speed=0;
		}		
		if ( speed > 0 && Robot.throwerBottomLimit.get() == true ) {
		//	speed=0;
		}		

		Robot.throwerClimberMotorLeft.set(speed*-1);
		Robot.throwerClimberMotorRight.set(speed);

        return(position);
	}

    /************************************************************************
	 ************************************************************************/

	public boolean setThrowerPosition(double angle) {
	
		double pos = Robot.throwerClimberMotorLeftRelativeEncoder.getPosition();
		double currAngle = pos / 42 / 500;

		boolean usePID=true;

		if (usePID) {
			if (reachedAngleTarget) {
				reachedAngleTarget=false;
				throwerPID.reset();
			}

			double speed = MathUtil.clamp(throwerPID.calculate(currAngle, angle),-0.2,.2);

			if (throwerPID.atSetpoint()) {
				reachedAngleCount++;
				moveThrower(0);
			} else {
				moveThrower(speed);
				reachedAngleCount=0;
				reachedAngleTarget=false;
			}
		} else {
			if (currAngle < angle -.5) {
				moveThrower(.3);
			} else if (currAngle > angle + .5) {
				moveThrower(-.3);
			} else {
				reachedAngleCount++;
				moveThrower(0);
			}		
		}	

		if (++reachedAngleCount>3) {
			reachedAngleTarget=true;
			return true;			
		} else {
			return false;
		}			

	}

    /************************************************************************
	 ************************************************************************/

    public void throwerTriggerOn() {
		Robot.throwerTriggerMotor.set(-1);
		Robot.pickup.runMotor(-1);
		Robot.triggerThrow=true;
	}

    /************************************************************************
	 ************************************************************************/

    public void throwerTriggerOff() {
		Robot.throwerTriggerMotor.set(0);
		Robot.pickup.runMotor(0);
		Robot.triggerThrow=false;
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
}


