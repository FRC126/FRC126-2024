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
//import frc.robot.RobotMap;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;


/**********************************************************************************
 **********************************************************************************/

public class Catapult extends SubsystemBase {
	int inversion=1;
	int limitHit=0;
	int searchForLimit=0;

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
		if ( pos < 5.2) {
   		    Robot.CatapultMotor.set(.8 * inversion);
		} else {
			cancel();
		}	
	}

	/************************************************************************
	 ************************************************************************/

     public void CatapultBackwards() { 
		double pos = getPos();
		double speed=-0.05;
		boolean limitActive;

		if (Robot.catapultBottomLimit.get() == true) {
			SmartDashboard.putBoolean("Cataput BottomLimit", true);
			limitHit=0;	 
			limitActive=false;
		} else {
			limitActive=true;
		    SmartDashboard.putBoolean("Cataput BottomLimit", false);
			if (speed < 0) { speed=0; }
			limitHit++;
			if (limitHit > 10) {
  			     Robot.CatapultRelativeEncoder.setPosition(-0.1);
				 limitHit=0;	 
			}
		}

		boolean searchingLimit = (pos <= 0 && !limitActive);

		if ( searchingLimit ) { 
			searchForLimit++;
			if ( searchForLimit > 15 ) {
				searchingLimit = false;
			}
		} else {
			searchForLimit=0;
		}

		if ( searchingLimit || pos > 0 || Robot.ignoreEncoders) {
	 	    Robot.CatapultMotor.set(speed);
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
		searchForLimit=0;
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