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

 public class Balance extends CommandBase {
    
    double pitch, xAxis, last_pitch, xAxisStart;
    int iters;
    int balanceCount=0;
    static final double balanceThresholdMin=-5;
    static final double balanceThresholdMax=5;

	/**********************************************************************************
	 **********************************************************************************/
	
    public Balance(int itersIn) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        iters=itersIn;
        balanceCount=0;
        pitch = Robot.navxMXP.getPitch();
        xAxisStart = Robot.navxMXP.getAngle();
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/
	
    public void initialize() {
        Robot.driveBase.brakesOn();      
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
    public void execute() {
        pitch = Robot.navxMXP.getPitch();
        xAxis = Robot.navxMXP.getAngle();
        double speed=0;
        
        SmartDashboard.putNumber("NavX Pitch",pitch);
        SmartDashboard.putNumber("Balance Count",balanceCount);

        if ( pitch > balanceThresholdMax) {
            // Pointing up, Drive forward
            speed=0.2;
            if ( pitch > 10) {
                speed=0.3;
            }
            balanceCount=0;
        } else if (pitch < balanceThresholdMin) {
            // Pointing down, Drive backwards
            speed=-0.2;
            if ( pitch < -10) {
                speed=0.3;
            }
            balanceCount=0;
        } else {
            // Reached target
            speed =0;
            balanceCount++;
            if (balanceCount == 1) {
                // Set the encoders to zero, so we can track any movement
                Robot.driveBase.resetEncoders();            
                Robot.driveBase.brakesOn();            
            } else {
                // Try to actively keep the robot in place
                double distance=Robot.driveBase.getDistanceInches();
                if (distance < -.5) { speed=.05; }
                if (distance > -.5) { speed=-0.5; }
            }
        }    

        SmartDashboard.putNumber("Balance Speed",speed);
        Robot.driveBase.Drive(speed, 0, true, xAxisStart);
     }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    public boolean isFinished() {
        iters--;
        if (balanceCount > 1000 || iters <= 0) {
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
        Robot.driveBase.cancel();
    }
}

