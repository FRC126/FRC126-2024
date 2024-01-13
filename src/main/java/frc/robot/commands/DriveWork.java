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

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;

/**********************************************************************************
 **********************************************************************************/

 public class DriveWork extends Command {
    double driveFb;
    double driveLr;
    double startAngle;
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
        startAngle = Robot.navxMXP.getAngle();
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
    public void execute() {
        iters--;

        if (driveLr == 0) {
            Robot.driveBase.Drive(driveFb, 0, true, startAngle);
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
