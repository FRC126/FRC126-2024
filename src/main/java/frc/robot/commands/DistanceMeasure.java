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
import frc.robot.subsystems.LidarLite;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**********************************************************************************
 **********************************************************************************/

public class DistanceMeasure extends CommandBase {

	/**********************************************************************************
	 **********************************************************************************/
	
	public DistanceMeasure(LidarLite subsystem) {
		// Use requires() here to declare subsystem dependencies
        addRequirements(subsystem);
    }     

	/**********************************************************************************
	 * Run before command starts 1st iteration
	 **********************************************************************************/
	
	@Override
	public void initialize() {
	}    

	/**********************************************************************************
	 **********************************************************************************/
	
	@Override
	public void execute() {
		Robot.distance.measureDistance();
    }

	/**********************************************************************************
	 * Returns true if command finished
	 **********************************************************************************/

	 @Override
	public boolean isFinished() {
		return false;
	}

	/**********************************************************************************
	 * Called once after isFinished returns true
	 **********************************************************************************/

	 @Override
	public void end(boolean isInterrupted) {
	}  
}
