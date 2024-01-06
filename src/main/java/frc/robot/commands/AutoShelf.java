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

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotMap;

/**********************************************************************************
 **********************************************************************************/

public class AutoShelf extends SequentialCommandGroup {
    public AutoShelf() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new ParallelCommandGroup(
               new MoveTowerArm(RobotMap.towerArmExtendedMidPos-7, 400),
               new MoveGrabber(RobotMap.grabberOpenPos, 150)
            ),

            new FinishAuto()
       );
    }       
}
