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

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;

/**********************************************************************************
 **********************************************************************************/

public class AutoPickupCone extends SequentialCommandGroup {
    public AutoPickupCone() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new MoveArmExtension(RobotMap.armExtendedPickupPos, 100),

            new ParallelCommandGroup(
                new MoveGrabber(RobotMap.grabberOpenPos, 150),
                new MoveTowerArm(RobotMap.towerArmConePickupPos, 100)
            ),

            new MoveGrabber(RobotMap.grabberConePos, 100),

            new ParallelCommandGroup(
                new MoveArmExtension(RobotMap.armRetractedPos, 100),
                new MoveTowerArm(RobotMap.towerArmRetractedPos, 100)
            ),    

            new FinishAuto()    
        );
    }       
}
