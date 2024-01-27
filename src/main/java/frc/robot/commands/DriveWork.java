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
    int itters = 500;

	/**********************************************************************************
	 **********************************************************************************/
	
    public DriveWork(double fb, double lr, double r, double dis) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        driveFb = fb;
        driveLr = lr;
        rotate = r;
        distance = dis;
        
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/
	
    public void initialize() {
        startAngle = Robot.navxMXP.getAngle();
        Robot.swerveDrive.brakesOn();
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
    public void execute() {
        itters--;
        if (driveLr == 0) {
            Robot.swerveDrive.Drive(driveFb, 0, 0, true, startAngle);
        } else {
            Robot.swerveDrive.Drive(driveFb, driveLr, 0);            
        }
     }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        
        SmartDashboard.putNumber("Distance Inches", Robot.swerveDrive.getDistanceInches());
        if (itters == 0 || distance <= Robot.swerveDrive.getDistanceInches()) {
            Robot.swerveDrive.Drive(0, 0, 0);
            Robot.swerveDrive.brakesOff();
            return true;
        }
        return false;
    }

	/**********************************************************************************
     * Called once after isFinished returns true
	 **********************************************************************************/
	
    public void end(boolean isInteruppted) {
        Robot.swerveDrive.Drive(0, 0, 0);
    }

}
