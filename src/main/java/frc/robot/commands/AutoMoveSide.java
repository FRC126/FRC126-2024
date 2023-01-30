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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

 public class AutoMoveSide extends CommandBase {  
    int direction;
    

	/**********************************************************************************
	 **********************************************************************************/
	
    public AutoMoveSide(int directionIn) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        direction=directionIn;
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/
	
    public void initialize() {
        Robot.driveBase.resetEncoders();       
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
    public void execute() {
     }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    public boolean isFinished() {
        return false;
    }

	/**********************************************************************************
     * Called once after isFinished returns true
	 **********************************************************************************/
	
    public void end(boolean isInteruppted) {
        Robot.driveBase.Drive(0, 0);
        Robot.isAutoMoveSide=false;
    }
}

