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

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

/**********************************************************************************
 **********************************************************************************/

public class AutoDrive extends SequentialCommandGroup {
    public AutoDrive(double forward, double leftRight, double rotate, double distanceDesired, int iters) {
        /**********************************************************************************
         **********************************************************************************/
        
        addCommands(
            new DriveWork(forward, leftRight, rotate, distanceDesired, iters),
            new InstantCommand(Robot.swerveDrive::resetEncoders, Robot.swerveDrive),
            new FinishAuto()
        );
    }
}
