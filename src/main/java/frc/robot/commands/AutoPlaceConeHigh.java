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

public class AutoPlaceConeHigh extends SequentialCommandGroup {
    public AutoPlaceConeHigh(int action) {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new DriveDistance(-6, 150),

            // Drive Backwards, Raise and Extend the Arm.
            new ParallelCommandGroup(
                new DriveDistance(-18, 400),
                new MoveTowerArm(RobotMap.towerArmExtendedHighPos, 400),
                new MoveArmExtension(RobotMap.armExtendedPlacePos, 400)
            ),

            // Drive Forwards
            new DriveDistance(26, 250),

            // Open the grabber to drop the cone
            new MoveGrabber(RobotMap.grabberConePos+50, 250),

            new ParallelCommandGroup(
                new DriveDistance(-12, 150),
                new MoveArmExtension(RobotMap.armExtendedPlacePos-100, 250)
            ),    

            // Drive backwards, lower and retract the arm, close the grabber
            new ParallelCommandGroup(
                new DriveDistance(-12, 250),
                new MoveArmExtension(RobotMap.armRetractedPos, 250),
                new MoveGrabber(RobotMap.grabberClosedPos+5, 250),
                new MoveTowerArm(RobotMap.towerArmRetractedPos, 250)
            ),    

            new FinishAuto()        
        );
        
        if (action == 1) {
            addCommands(
                new TurnDegreesBetter(180,250),
                
                new AutoClimbBalance()
            );
        }
        if (action == 2) {
            addCommands(
                new DriveDistance(-12*8,300),

                new TurnDegreesBetter(180,250)
            );
        }
    }       
}
