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

public class AutoPlaceConeMid extends SequentialCommandGroup {
    public AutoPlaceConeMid(int action) {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new DriveDistance(-6, 150),

            new ParallelCommandGroup(
                new DriveDistance(-18, 150),
                new MoveTowerArm(RobotMap.towerArmExtendedMidPos, 400)
            ),

            new DriveDistance(15, 250),

            new MoveGrabber(RobotMap.grabberConePos+50, 250),

            new DriveDistance(-10, 250),
            
            new ParallelCommandGroup(
                new DriveDistance(-1, 250),
                new MoveGrabber(RobotMap.grabberConePos, 250),
                new MoveTowerArm(RobotMap.towerArmRetractedPos+5, 250)
            ),    

            new FinishAuto()        
        );

        if (action == 1) {
            addCommands(
                new TurnDegreesBetter(180,250),
                
                new AutoClimbBalance()
            );
        }

    }       
}
