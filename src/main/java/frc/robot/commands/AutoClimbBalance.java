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

 public class AutoClimbBalance extends CommandBase {
    
    double pitch, xAxis, last_pitch, xAxisStart;
    boolean climbed=false;
    int iters;
    int balanceCount=0;
    static final double balanceThresholdMin=-5;
    static final double balanceThresholdMax=5;

	/**********************************************************************************
	 **********************************************************************************/
	
    public AutoClimbBalance() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        iters=300;
        balanceCount=0;
        pitch = Robot.navxMXP.getPitch();
        xAxisStart = Robot.navxMXP.getAngle();
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
        pitch = Robot.navxMXP.getPitch();
        xAxis = Robot.navxMXP.getAngle();
        double speed=0;
        double rotate=0;

        SmartDashboard.putNumber("NavX Pitch",pitch);
        SmartDashboard.putNumber("Balance Count",balanceCount);
        SmartDashboard.putBoolean("Climbed",climbed);

        if (!climbed) {
            speed=0.3;
            if (pitch > 8) {
                climbed=true;
            }
        } else {
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
                balanceCount++;
            }    
        }

        SmartDashboard.putNumber("Balance Speed",speed);
        if (speed == 0) {
            Robot.driveBase.brakesOn();
        } else {
            Robot.driveBase.brakesOff();
        }
        Robot.driveBase.Drive(speed, rotate, true, xAxisStart);

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
        Robot.driveBase.cancel();
    }
}

