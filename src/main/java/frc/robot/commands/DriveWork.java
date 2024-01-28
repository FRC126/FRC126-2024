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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DriveWork extends Command {
    double driveFb;
    double driveLr;
    double startAngle;
    double rotate;
    double distance;
    int iters;
    int distanceReached;
    boolean driveWorkDebug=true;

	/**********************************************************************************
	 **********************************************************************************/
	
    public DriveWork(double fb, double lr, double r, double dis, int itersIn) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        driveFb = fb;
        driveLr = lr;
        rotate = r;
        distance = dis;
        iters = itersIn;
        distanceReached=0;
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/

    @Override
    public void initialize() {
        startAngle = Robot.navxMXP.getAngle();
        Robot.swerveDrive.brakesOn();
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
	@Override
    public void execute() {
        double FB = driveFb,
               LR = driveLr;
        iters--;

        double tmp = Robot.swerveDrive.getDistanceInches();

        if (tmp + 6 > distance) {
            // Slow down as we get close to the distance
            FB=driveFb*.5;
            LR=driveLr*.5;
        } else if (tmp + 3 > distance) {
            // Slow down as we get close to the distance
            FB=driveFb*.25;
            LR=driveLr*.25;
        } else if (tmp + 1 > distance) {
            // Within one inch of the target
            Robot.swerveDrive.brakesOn();
            FB=0;
            LR=0;
            distanceReached++;
        }

        if (driveWorkDebug) { 
            SmartDashboard.putNumber("Distance Inches", tmp);
            SmartDashboard.putNumber("Distance Reached", distanceReached);
        }    

        Robot.swerveDrive.Drive(FB, LR, 0);            
     }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    // Make this return true when this Command no longer needs to run execute()
	@Override
    public boolean isFinished() {
        
        if (iters == 0 || distanceReached > 5) {
            Robot.swerveDrive.Drive(0, 0, 0);
            Robot.swerveDrive.brakesOff();
            return true;
        }
        return false;
    }

	/**********************************************************************************
     * Called once after isFinished returns true
	 **********************************************************************************/
	
	@Override
    public void end(boolean isInteruppted) {
        Robot.swerveDrive.Drive(0, 0, 0);
        Robot.swerveDrive.brakesOff();
    }

}
