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

 public class DriveDistance extends CommandBase {
    double driveFb;
    double driveLr;
    double targetAngle;
    double distance;
    int iters;

	/**********************************************************************************
	 **********************************************************************************/
	
    public DriveDistance(double fb, double lr, double distance_in, int iters_in ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        driveFb = fb;
        driveLr = lr;
        distance = distance_in;
        iters = iters_in;
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/
	
    public void initialize() {
        targetAngle = Robot.internalData.getGyroAngle();
        Robot.driveBase.resetEncoders();
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
    public void execute() {
        boolean useGyro=true;

        if(driveLr == 0 && useGyro == true) {
            if(Robot.internalData.getGyroAngle() - targetAngle > 1) {
                // We are drifiting to the left, correct
                Robot.driveBase.Drive(driveFb, -0.1);
            }
            else if(Robot.internalData.getGyroAngle() - targetAngle < -1) {
                // We are drifiting to the right, correct
                Robot.driveBase.Drive(driveFb, 0.1);
            } else {
                // Drive straight
                Robot.driveBase.Drive(driveFb, 0);
            }
        } else {
            // ignore the gyro
            Robot.driveBase.Drive(driveFb, driveLr);
            
        }
     }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    public boolean isFinished() {
        iters--;
        double currentDistance = Robot.driveBase.getDistanceInches();
        if (currentDistance >= distance || iters <= 0) {
            // if we have reached the target distance, or run out of time to do so, 
            // stop driving and end the command.
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

