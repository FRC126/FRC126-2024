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
    double distanceDesired;
    int iters;
    double distanceReached = 0;
    boolean driveWorkDebug=true;

	/**********************************************************************************
	 **********************************************************************************/
	
    public DriveWork(double fb, double lr, double r, double distanceDesired, int itersIn) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.driveFb = fb;
        this.driveLr = lr;
        this.rotate = r;
        this.distanceDesired = distanceDesired;
        this.iters = itersIn;
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/

    @Override
    public void initialize() {
        distanceDesired = SmartDashboard.getNumber(Robot.DISTANCE_DESIRED, distanceDesired);
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
        distanceReached = Robot.swerveDrive.getDistanceInches();

        if (distanceReached + 5 > distanceDesired) {
            // Slow down as we get close to the distance
            FB=driveFb*.5;
            LR=driveLr*.5;
            SmartDashboard.putBoolean(Robot.WITHIN_FIVE, true);
        } 

        if (driveWorkDebug) { 
            SmartDashboard.putNumber(Robot.DISTANCE_ACHIEVED, distanceReached);
        }    

        Robot.swerveDrive.Drive(FB, LR, 0);            
     }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    // Make this return true when this Command no longer needs to run execute()
	@Override
    public boolean isFinished() {
        if (iters == 0 || distanceReached >= distanceDesired) {
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
