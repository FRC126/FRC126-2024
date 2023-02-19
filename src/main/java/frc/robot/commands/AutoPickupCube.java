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
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;

 
/**********************************************************************************
 **********************************************************************************/

public class AutoPickupCube extends SequentialCommandGroup {
    public AutoPickupCube() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new MoveArmExtension(ArmExtension.armExtendedPickupPos, 250),

            new MoveGrabber(Grabber.grabberCubePos, 250),

            new MoveArmExtension(ArmExtension.armRetractedPos, 250),

            new MoveTowerArm(TowerArm.armRetractedPos, 250),

            new FinishAuto()        );
    }       
}
