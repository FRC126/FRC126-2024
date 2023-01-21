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
import edu.wpi.first.wpilibj2.command.CommandBase;

/**********************************************************************************
 **********************************************************************************/

public class IntakeWork extends CommandBase {
    boolean forward;
    int iters;
    int count=0;

	/**********************************************************************************
	 **********************************************************************************/
	
    public IntakeWork(boolean forward_in, int iters_in) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        forward = forward_in;
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
        count--;
        if(forward) {
            Robot.ballIntake.IntakeRun();
        } else {    
            Robot.ballIntake.IntakeSpeed(-0.7);
        }    
     }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    public boolean isFinished() {
        if (count <= 0) {
            Robot.ballIntake.IntakeSpeed(0.0);
            return true;
        }
        return false;
    }

	/**********************************************************************************
     * Called once after isFinished returns true
	 **********************************************************************************/
	
    public void end(boolean isInteruppted) {
        Robot.ballIntake.IntakeSpeed(0.0);
    }

}

