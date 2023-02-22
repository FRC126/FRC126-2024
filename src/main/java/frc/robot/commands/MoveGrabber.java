/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/
    Team 126 2022 Code       
	Go get em gaels!
***********************************/

package frc.robot.commands;

import frc.robot.Robot;
//import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

 public class MoveGrabber extends CommandBase {
    double target;
    int iters;
    int targetReached=0;

	/**********************************************************************************
	 **********************************************************************************/
	
    public MoveGrabber(double targetIn, int iters_in ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        target = targetIn;
        iters = iters_in;
        targetReached = 0;
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

        double curPos=Robot.robotGrabber.getPos();
        double targetPos=target;
        double speed=0;

        if (curPos < targetPos - 4) { 
            speed=-0.8;
            if (curPos > targetPos -20) { speed=- 0.4; }
            targetReached=0;
        } else if (curPos > targetPos + 4) { 
            speed=0.8;
            if (curPos < targetPos +20) { speed=-0.4; }
            targetReached=0;
        } else {
            speed=0;
            targetReached++;
        }
        Robot.robotGrabber.MoveGrabber(speed);
    }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    public boolean isFinished() {
        iters--;

        if (targetReached > 2 || iters <= 0) {
            // We have reached our target angle or run out of time to do so.
            Robot.robotGrabber.cancel();
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