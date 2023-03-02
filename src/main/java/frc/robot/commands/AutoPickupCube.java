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

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;
import frc.robot.Robot;

 
/**********************************************************************************
 **********************************************************************************/

public class AutoPickupCube extends SequentialCommandGroup {
    public AutoPickupCube() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new InstantCommand(Robot.robotFlap::RetractFlap, Robot.robotFlap),

            new MoveGrabber(310, 250),
                      
            new MoveTowerArm(12, 250),

            new MoveArmExtension(RobotMap.armExtendedPickupPos-2, 75),

            new MoveGrabber(170, 250),

            new MoveArmExtension(RobotMap.armRetractedPos, 250),

            new FinishAuto()
        );
    }       
}
