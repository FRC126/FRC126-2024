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

public class AutoPlaceCubeHigh extends SequentialCommandGroup {
    public AutoPlaceCubeHigh() {
        /**********************************************************************************
         **********************************************************************************/

         addCommands(
            new DriveDistance(-12, 150),

            new ParallelCommandGroup(
                new DriveDistance(-12, 150),
                new MoveTowerArm(RobotMap.towerArmExtendedHighPos-40, 400),
                new MoveArmExtension(RobotMap.armExtendedPlacePos, 400)
            ),

            new DriveDistance(25, 250),

            new MoveGrabber(RobotMap.grabberOpenPos, 250),

            new ParallelCommandGroup(
                new DriveDistance(-12, 150),
                new MoveArmExtension(RobotMap.armRetractedPos, 250),
                new MoveGrabber(RobotMap.grabberConePos, 250)
            ),    

            new ParallelCommandGroup(
                new DriveDistance(-12, 250),
                new MoveTowerArm(RobotMap.towerArmRetractedPos, 250)
            ),    

            new FinishAuto()        
        );
    }       
}
