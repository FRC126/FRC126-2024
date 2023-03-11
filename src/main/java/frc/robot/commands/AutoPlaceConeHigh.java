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
                new DriveDistance(-12, 400),
                new MoveTowerArm(RobotMap.towerArmExtendedHighPos-5, 400),
                new MoveArmExtension(RobotMap.armExtendedPlacePos, 400)
            ),

            // Drive Forwards
            new DriveDistance(20, 200),

            new MoveTowerArm(RobotMap.towerArmExtendedHighPos-20, 200),

            // Open the grabber to drop the cone
            new MoveGrabber(RobotMap.grabberConePos+100, 100),

            new ParallelCommandGroup(
                new DriveDistance(-12, 150),
                new MoveArmExtension(RobotMap.armExtendedPlacePos-100, 200)
            ),    

            // Drive backwards, lower and retract the arm, close the grabber
            new ParallelCommandGroup(
                new DriveDistance(-6, 100),
                new MoveArmExtension(RobotMap.armRetractedPos, 100),
                new MoveGrabber(RobotMap.grabberClosedPos+5, 100),
                new MoveTowerArm(RobotMap.towerArmRetractedPos, 125)
            ),    

            //new DriveDistance(12, 75),

            new FinishAuto()        
        );
        
        if (action == 1) {
            addCommands(
                //new TurnDegreesBetter(175,250),
                
                new AutoClimbBalanceBackwards()
            );
        }
        if (action == 2) {
            addCommands(
                new DriveDistance(-16*8,300)

                //new TurnDegreesBetter(175,250)
            );
        }
        if (action == 3) {
            addCommands(
                new TurnDegreesBetter(-15,150),
                new DriveDistance(-18,300),
                new TurnDegreesBetter(15,150),
                new DriveDistance(-12*10,400)
                //new TurnDegreesBetter(175,250)
            );
        }
    }       
}
