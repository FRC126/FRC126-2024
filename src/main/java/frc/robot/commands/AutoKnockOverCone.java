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
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**********************************************************************************
 **********************************************************************************/

public class AutoKnockOverCone extends SequentialCommandGroup {
    public AutoKnockOverCone() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new ParallelCommandGroup(
                new MoveGrabber(RobotMap.grabberOpenPos, 250),
                new MoveTowerArm(RobotMap.towerArmFloorPickupPos, 250),
                new MoveArmExtension(RobotMap.armRetractedPos, 250)
            ),    

            new FinishAuto()        
        );
    }       
}
