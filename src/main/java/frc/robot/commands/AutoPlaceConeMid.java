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
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;
 
/**********************************************************************************
 **********************************************************************************/

public class AutoPlaceConeMid extends SequentialCommandGroup {
    public AutoPlaceConeMid() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new DriveDistance(-12, 150),
/*
            //new ParallelCommandGroup(
            //    new MoveTowerArm(TowerArm.armExtendedMidPos, 250),
            //    new MoveArmExtension(ArmExtension.armExtendedPlacePos, 250)
            //),

            new MoveTowerArm(TowerArm.armExtendedMidPos/2, 400),

            new ParallelCommandGroup(
                new MoveTowerArm(TowerArm.armExtendedMidPos, 400),
                new MoveArmExtension(ArmExtension.armExtendedPlacePos, 400)
            ),

            //new DriveDistance(12, 150),

            new MoveGrabber(Grabber.grabberCubePos/2, 400),

            //new DriveDistance(-12, 150),

            new ParallelCommandGroup(
                new MoveArmExtension(ArmExtension.armRetractedPos , 400),
                new MoveGrabber(Grabber.grabberConePos, 400)
            ),    

            new MoveTowerArm(TowerArm.armPickupPos, 400),
*/
            new FinishAuto()
        );
    }       
}
