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

public class AutoPlaceCubeMid extends SequentialCommandGroup {
    public AutoPlaceCubeMid() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new DriveDistance(-6, 150),

            new ParallelCommandGroup(
                new DriveDistance(-18, 150),
                new MoveTowerArm(RobotMap.towerArmExtendedMidPos, 400)
                //new MoveArmExtension(ArmExtension.armExtendedPlacePos, 400)
            ),

            new DriveDistance(15, 250),

            new MoveGrabber(RobotMap.grabberOpenPos, 250),

            new DriveDistance(-10, 250),
            
            new ParallelCommandGroup(
                new DriveDistance(-8, 250),
                //new MoveArmExtension(ArmExtension.armRetractedPos, 250),
                new MoveGrabber(RobotMap.grabberConePos, 250),
                new MoveTowerArm(RobotMap.towerArmRetractedPos+5, 250)
            ),    
            new FinishAuto()        
        );
    }       
}
