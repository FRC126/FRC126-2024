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
                new DriveDistance(-12, 150),
                new MoveTowerArm(RobotMap.towerArmExtendedMidPos-5, 400)
            ),

            new DriveDistance(9, 250)
        );

        if (action != 0) {
            // During Auto
            addCommands(
                new MoveTowerArm(RobotMap.towerArmExtendedMidPos-20, 400),

                new MoveGrabber(RobotMap.grabberConePos+100, 250),

                new DriveDistance(-10, 250),
                
                new ParallelCommandGroup(
                    new DriveDistance(-1, 250),
                    new MoveGrabber(RobotMap.grabberConePos, 250),
                    new MoveTowerArm(RobotMap.towerArmRetractedPos+5, 250)
                )
            );    
        } else {
            // User Command
            addCommands(
                new MoveTowerArm(RobotMap.towerArmExtendedMidPos-15, 400)
            );    
        }

        if (action == 1) {
            addCommands(
                new TurnDegreesBetter(175,250),
                
                new AutoClimbBalance()
            );
        }
        if (action == 2) {
            addCommands(
                new DriveDistance(-16*8,300)

                //new TurnDegreesBetter(175,250)
            );
        }

        addCommands(
            new FinishAuto()        
        );    

    }       
}
