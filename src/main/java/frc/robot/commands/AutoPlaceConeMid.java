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
            new ParallelCommandGroup(
                new DriveDistance(-24, 150),
                new MoveTowerArm(TowerArm.armExtendedMidPos, 400),
                new MoveArmExtension(ArmExtension.armExtendedPlacePos, 400)
            ),

            new DriveDistance(6, 250),

            new MoveGrabber(Grabber.grabberConePos+50, 250),

            new ParallelCommandGroup(
                new DriveDistance(-6, 250),
                new MoveArmExtension(ArmExtension.armRetractedPos, 250),
                new MoveGrabber(Grabber.grabberConePos, 250),
                new MoveTowerArm(TowerArm.armRetractedPos+5, 250)
            ),    

            new FinishAuto()        
        );
    }       
}