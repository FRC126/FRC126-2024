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

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

 public class AutoBalance extends CommandBase {
    
    double pitch;
    int iters;
    int last_pitch;
    int balanceCount=0;
    static final double balanceThresholdMin=-5;
    static final double balanceThresholdMax=5;

	/**********************************************************************************
	 **********************************************************************************/
	
    public AutoBalance( ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        iters=200;
        balanceCount=0;
        pitch = Robot.ahrs.getPitch();
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/
	
    public void initialize() {
        Robot.driveBase.resetEncoders();
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
    public void execute() {
        pitch = Robot.ahrs.getPitch();
        double speed=0;

        SmartDashboard.putNumber("NavX Pitch",pitch);
        SmartDashboard.putNumber("Balance Count",balanceCount);

        if ( pitch > balanceThresholdMax) {
            // Still pointup, Drive forward
            Robot.driveBase.brakesOff();
            speed=0.15;
            if ( pitch > 10) {
                speed=0.25;
            }
            Robot.driveBase.Drive(speed, 0);
            balanceCount=0;
            SmartDashboard.putNumber("Balance Speed",speed);
        } else if (pitch < balanceThresholdMin) {
            // Pointing down, Drive backwards
            Robot.driveBase.brakesOff();
            speed=-0.15;
            if ( pitch < -10) {
                speed=0.25;
            }
            Robot.driveBase.Drive(speed, 0);
            balanceCount=0;
            SmartDashboard.putNumber("Balance Speed",speed);
        } else {
            // Reached target
            balanceCount++;
            Robot.driveBase.Drive(0, 0);
            Robot.driveBase.brakesOn();
        }    
     }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    public boolean isFinished() {
        iters--;
        if (balanceCount > 5 || iters <= 0) {
            // if we have reached the target distance, or run out of time to do so, 
            // stop driving and end the command.
            Robot.driveBase.Drive(0, 0);
            Robot.driveBase.brakesOn();
            return true;
        }
        return false;
    }

	/**********************************************************************************
     * Called once after isFinished returns true
	 **********************************************************************************/
	
    public void end(boolean isInteruppted) {
        Robot.driveBase.Drive(0, 0);
    }
}

