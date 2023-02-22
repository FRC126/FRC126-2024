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
import frc.robot.subsystems.*;

//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.Robot;
 
/**********************************************************************************
 **********************************************************************************/

public class AutoPlaceConeHigh extends SequentialCommandGroup {
    public AutoPlaceConeHigh() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            // Drive Backwards, Raise and Extend the Arm.
            new ParallelCommandGroup(
                new DriveDistance(-24, 150),
                new MoveTowerArm(TowerArm.armExtendedHighPos, 400),
                new MoveArmExtension(ArmExtension.armExtendedPlacePos, 400)
            ),

            // Drive Forwards
            new DriveDistance(25, 250),

            // Open the grabber to drop the cone
            new MoveGrabber(Grabber.grabberConePos+50, 250),

            // Drive backwards, lower and retract the arm, close the grabber
            new ParallelCommandGroup(
                new DriveDistance(-24, 250),
                new MoveArmExtension(ArmExtension.armRetractedPos, 250),
                new MoveGrabber(Grabber.grabberConePos, 250),
                new MoveTowerArm(TowerArm.armRetractedPos, 250)
            ),    

            new FinishAuto()        
        );
    }       
}
