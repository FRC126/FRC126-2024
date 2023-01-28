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

/**********************************************************************************
 **********************************************************************************/

 public class DriveWork extends CommandBase {
    double driveFb;
    double driveLr;
    double targetAngle;
    int iters;

	/**********************************************************************************
	 **********************************************************************************/
	
    public DriveWork(double fb, double lr, int iters_in) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        driveFb = fb;
        driveLr = lr;
        iters = iters_in;
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/
	
    public void initialize() {
        targetAngle = Robot.internalData.getGyroAngle();
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
    public void execute() {
        iters--;
        boolean useGyro=false;

        if(driveLr == 0 && useGyro == true) {
            if(Robot.internalData.getGyroAngle() - targetAngle > 1) {
                Robot.driveBase.Drive(driveFb, -0.1);
            }
            else if(Robot.internalData.getGyroAngle() - targetAngle < -1) {
                Robot.driveBase.Drive(driveFb, 0.1);
            } else {
                Robot.driveBase.Drive(driveFb, 0);
            }
        } else {
            Robot.driveBase.Drive(driveFb, driveLr);            
        }
     }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        if (iters <= 0) {
            Robot.driveBase.Drive(0, 0);
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
