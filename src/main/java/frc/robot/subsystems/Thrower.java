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

import com.ctre.phoenix6.*;

/**********************************************************************************
 **********************************************************************************/

public class Thrower extends SubsystemBase {	
    static double targetRPM;
	static double throwerSpeed[] = { 0,0,0 };
    static int delay;
    //static double P = 0.000008;
    static double P = 0.000025;
    static double I = -0.0003;
	boolean throwerDebug=true;
	public static double myRPM=3800;

	/************************************************************************
	 ************************************************************************/

	public Thrower() {

		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new ThrowerControl(this));
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

	/************************************************************************
     * Run Main Thower Wheels by target RPM
	 ************************************************************************/

    public boolean throwerRPM(int index, double targetRPM) {
        boolean targetReached=false;
		double ix, error=0.0, rpm;
		StatusSignal RPM;

		if (index == 1) {
		    RPM = Robot.throwerTalonOne.getVelocity();
 			rpm = RPM.getValueAsDouble() * 60;
		} else {
		    RPM = Robot.throwerTalonTwo.getVelocity();
			rpm = RPM.getValueAsDouble() * 60 * -1;
		}	

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

		if (targetRPM < rpm + 75 && targetRPM > rpm - 75) {
			targetReached=true;
		}

        // Set the speed on the Thrower Motors
		if (index == 1) {
			Robot.throwerTalonOne.set(throwerSpeed[index]);
		} else {
  			Robot.throwerTalonTwo.set(throwerSpeed[index] * -1);
		}	

		if (throwerDebug) {
			// Log info to the smart dashboard
			String foo="Thrower " + index + " RPM Current";
			SmartDashboard.putNumber(foo,rpm);
			foo="Thrower " + index + " RPM Target";
			SmartDashboard.putNumber(foo,targetRPM);
			foo="Thrower " + index + " RPM Reached";
			SmartDashboard.putBoolean(foo,targetReached);
			foo="Thrower " + index + " speed";
			SmartDashboard.putNumber(foo,throwerSpeed[index]);
		}

        return(targetReached);
    }

    /************************************************************************
	 ************************************************************************/

    public double moveThrower(double speed) {
        double position=0;

		double left = Robot.throwerClimberMotorLeftRelativeEncoder.getPosition();
		double right = Robot.throwerClimberMotorRightRelativeEncoder.getPosition()*-1;

		position=left+right/2.0;

		if ( speed < 0 && Robot.throwerTopLimit.get() == true ) {
			speed=0;
		}		
		if ( speed > 0 && Robot.throwerTopLimit.get() == true ) {
			speed=0;
		}		

		Robot.throwerClimberMotorLeft.set(speed);
		Robot.throwerClimberMotorRight.set(speed*-1);

        return(position);
	}


    /************************************************************************
	 ************************************************************************/

	public void cancel() {
        throwerRPM(1,0); 
        throwerRPM(2,0); 
	}

	public double getRPM() {
		return myRPM;
	}

	public void setRPM(double rpmIn) {
		myRPM = rpmIn;
	}
}

