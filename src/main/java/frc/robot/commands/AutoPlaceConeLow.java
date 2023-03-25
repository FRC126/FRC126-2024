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

public class AutoPlaceConeLow extends SequentialCommandGroup {
    public AutoPlaceConeLow(int action) {
        /**********************************************************************************
         **********************************************************************************/

         addCommands(
            new ParallelCommandGroup(
                new MoveTowerArm(RobotMap.towerArmExtendedLowPos, 250),
                new MoveArmExtension(RobotMap.armExtendedPlaceLow, 250)
            ),

            new MoveGrabber(RobotMap.grabberOpenPos, 150),
         
            new DriveDistance(-12, 100),

            new ParallelCommandGroup(
                new MoveArmExtension(RobotMap.armRetractedPos, 150),
                new MoveGrabber(RobotMap.grabberConePos, 150),
                new MoveTowerArm(RobotMap.towerArmRetractedPos, 150)
            )
        );

        if (action == 1) {
            addCommands(
                new AutoClimbBalanceBackwards()
            );
        }
        if (action == 2) {
            addCommands(
                new DriveDistance(-16*8,300),

                new TurnDegreesBetter(180,250)
            );
        }

        addCommands(
            new FinishAuto()
        );    
    }        
}
