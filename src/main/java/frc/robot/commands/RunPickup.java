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
//import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

 public class RunPickup extends CommandBase {
    int direction;
    int iters;

	/**********************************************************************************
	 **********************************************************************************/
	
    public RunPickup(int directionIn, int iters_in ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        direction = directionIn;
        iters = iters_in;
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/
	
    public void initialize() {
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
    public void execute() {

        if (direction == 1) {
            Robot.robotPickup.pickupIntake();
            Robot.robotPickup.DeployPickup();
        } else {
            Robot.robotPickup.pickupEject();
            Robot.robotPickup.DeployPickup();
        }
    }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    public boolean isFinished() {
        iters--;

        if (iters <= 0) {
            // We have reached our target angle or run out of time to do so.
            Robot.robotPickup.cancel();
            return true;
        }

        return false;
    }

	/**********************************************************************************
     * Called once after isFinished returns true
	 **********************************************************************************/
	
    public void end(boolean isInteruppted) {
        Robot.robotGrabber.cancel();
    }
}