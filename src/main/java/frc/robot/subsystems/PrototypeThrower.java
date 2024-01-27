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

public class PrototypeTalon extends SubsystemBase {	
	boolean protoDebug=true;
    static double targetRPM;
	static double throwerSpeed[] = { 0,0 };
    static int delay;
    static double P = 0.000008;
    static double I = -0.0003;

	/************************************************************************
	 ************************************************************************/

	public PrototypeTalon() {

		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new PrototypeThrowerControl(this));
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

    /************************************************************************
	 ************************************************************************/

	public void runMotors(double speed) {
		double motorOneRPM, motorTwoRPM;

		Robot.protoTalonOne.set(speed*.7);
		Robot.protoTalonTwo.set(speed*.7*-1);

		StatusSignal OneRPM = Robot.protoTalonOne.getVelocity();
		StatusSignal TwoRPM = Robot.protoTalonTwo.getVelocity();

		motorOneRPM = OneRPM.getValueAsDouble() * 60;
		motorTwoRPM = TwoRPM.getValueAsDouble() * 60;
		
		if (protoDebug) {
			SmartDashboard.putNumber("Proto Talon One RPM",motorOneRPM);
			SmartDashboard.putNumber("Proto Talon Two RPM",motorTwoRPM);
		}
		
	}

	/************************************************************************
     * Run Main Thower Wheels by target RPM
	 ************************************************************************/

    public boolean throwerRPM(int index, double targetRPM) {
        boolean targetReached=false;
		double ix, error=0.0, rpm;
		StatusSignal RPM;

		if (index == 1) {
		    RPM = Robot.protoTalonOne.getVelocity();
 			rpm = RPM.getValueAsDouble() * 60;
		} else {
		    RPM = Robot.protoTalonTwo.getVelocity();
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
			Robot.protoTalonOne.set(throwerSpeed[index]);
		} else {
  			Robot.protoTalonTwo.set(throwerSpeed[index] * -1);
		}	

        // Log info to the smart dashboard
		String foo="Thrower " + index + " RPM Current";
		SmartDashboard.putNumber(foo,rpm);
		foo="Thrower " + index + " RPM Target";
        SmartDashboard.putNumber(foo,targetRPM);
		foo="Thrower " + index + " RPM Reached";
        SmartDashboard.putBoolean(foo,targetReached);

        return(targetReached);
    }

    /************************************************************************
	 ************************************************************************/

	public void cancel() {
        runMotors(0); 
	}
}

