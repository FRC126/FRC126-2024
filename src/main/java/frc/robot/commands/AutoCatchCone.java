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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotMap;
import frc.robot.Robot;

/**********************************************************************************
 **********************************************************************************/

public class AutoCatchCone extends SequentialCommandGroup {
    public AutoCatchCone() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new MoveArmExtension(RobotMap.armRetractedPos, 250),
          
            new ParallelCommandGroup(
                new MoveGrabber(RobotMap.grabberOpenPos, 150),
                new MoveTowerArm(RobotMap.towerArmRetractedPos, 150)
            ),

            new FinishAuto()
       );
    }       
}
