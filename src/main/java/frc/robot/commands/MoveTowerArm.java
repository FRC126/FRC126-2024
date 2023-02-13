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
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

 public class MoveTowerArm extends CommandBase {
    int target;
    int iters;
    int targetReached=0;

	/**********************************************************************************
	 **********************************************************************************/
	
    public MoveTowerArm(int targetIn, int iters_in ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        target = targetIn;
        iters = iters_in;
        targetReached=0;
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

        double curPos=Robot.robotTowerArm.getPos();
        double targetPos=0;

        switch (target) {
            case 0:
                targetPos=TowerArm.armRetractedPos;
                break;
            case 1:
                targetPos=TowerArm.armPickupPos;
                break;
            case 2:    
                targetPos=TowerArm.armExtendedMidPos;
                break;
            case 3:    
                targetPos=TowerArm.armExtendedHighPos;
                break;
        }

        if (curPos < targetPos - .1) { 
            Robot.robotTowerArm.MoveArm(.3);
        } else if (curPos > targetPos + .1) { 
            Robot.robotTowerArm.MoveArm(-0.3);
        } else {
            Robot.robotTowerArm.MoveArm(0);
        }

    }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    public boolean isFinished() {
        iters--;

        if (targetReached > 2 || iters <= 0) {
            // We have reached our target angle or run out of time to do so.
            Robot.robotTowerArm.MoveArm(0);
            return true;
        }

        return false;
    }

	/**********************************************************************************
     * Called once after isFinished returns true
	 **********************************************************************************/
	
    public void end(boolean isInteruppted) {
        Robot.robotTowerArm.MoveArm(0);
    }
}