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
                new MoveTowerArm(RobotMap.towerArmExtendedLowPos, 400),
                new MoveArmExtension(RobotMap.armExtendedPlaceLow, 400)
            ),

            new MoveGrabber(RobotMap.grabberOpenPos, 250),
            new MoveArmExtension(RobotMap.armRetractedPos, 400),
            new MoveTowerArm(RobotMap.towerArmExtendedLowPos+30, 250),
            

            new DriveDistance(-12, 150),

            new ParallelCommandGroup(
                new MoveArmExtension(RobotMap.armRetractedPos, 250),
                new MoveGrabber(RobotMap.grabberConePos, 250),
                new MoveTowerArm(RobotMap.towerArmRetractedPos, 250)
            ),

            //new MoveGrabber(Grabber.grabberOpenPos-20, 250),

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
                new DriveDistance(-16*8,300),

                new TurnDegreesBetter(180,250)
            );
        }
    }        
}
