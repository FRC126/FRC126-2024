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


/**********************************************************************************
 **********************************************************************************/

public class Catapult extends SubsystemBase {
	int inversion=1;

	/************************************************************************
	 ************************************************************************/

	public Catapult() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new CatapultControl(this));
       
    	   Robot.CatapultMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		   resetEncoders();
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

	/************************************************************************
	 ************************************************************************/

	public void CatapultForward() { 
		double pos = getPos();
		if ( pos < 4.5) {
   		    Robot.CatapultMotor.set(.7 * inversion);
		} else {
			cancel();
		}	
	}

	/************************************************************************
	 ************************************************************************/

     public void CatapultBackwards() { 
		double pos = getPos();
		if ( pos > 0 || Robot.ignoreEncoders) {
	 	    Robot.CatapultMotor.set(-.05 * inversion);
			if (Robot.ignoreEncoders){
				resetEncoders();
			}
		} else {
			cancel();
		}	
	}
    /************************************************************************
	 ************************************************************************/

	 public void cancel() {
		Robot.CatapultMotor.set(0);
		Robot.CatapultMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}


    /************************************************************************
	 ************************************************************************/

	 public void resetEncoders() {
		// Need to use encoders for the NEOs
		Robot.CatapultRelativeEncoder.setPosition(0);
	}

    /************************************************************************
	 ************************************************************************/

	 public double getPos() {
		// Need to use encoders for the NEOs
		double pos = Robot.CatapultRelativeEncoder.getPosition() * inversion;
		SmartDashboard.putNumber("Catapult POS", pos);
		return(pos);
		
	}

}