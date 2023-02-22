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
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;
 
/**********************************************************************************
 **********************************************************************************/

public class AutoPlaceConeLow extends SequentialCommandGroup {
    public AutoPlaceConeLow() {
        /**********************************************************************************
         **********************************************************************************/

         addCommands(
            new ParallelCommandGroup(
                new MoveTowerArm(TowerArm.armExtendedLowPos, 400),
                new MoveArmExtension(ArmExtension.armExtendedPlaceLow, 400)
            ),

            new MoveGrabber(Grabber.grabberConePos+50, 250),

            new ParallelCommandGroup(
                new MoveArmExtension(ArmExtension.armRetractedPos, 250),
                new MoveGrabber(Grabber.grabberConePos, 250),
                new MoveTowerArm(TowerArm.armRetractedPos, 250)
            ),    

            new FinishAuto()
        );
    }        
}