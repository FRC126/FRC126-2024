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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;
import frc.robot.Robot;

/**********************************************************************************
 **********************************************************************************/

public class AutoCatchCube extends SequentialCommandGroup {
    public AutoCatchCube() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new MoveArmExtension(RobotMap.armRetractedPos, 250),

            new ParallelCommandGroup(
                new MoveGrabber(RobotMap.grabberCubePos-50, 250),
                new MoveTowerArm(20, 250)
            ),

            new InstantCommand(Robot.robotFlap::DeployFlap, Robot.robotFlap),

            new FinishAuto()
       );
    }       
}
