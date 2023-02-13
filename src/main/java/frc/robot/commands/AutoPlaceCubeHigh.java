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
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.Robot;
 
/**********************************************************************************
 **********************************************************************************/

public class AutoPlaceCubeHigh extends SequentialCommandGroup {
    public AutoPlaceCubeHigh() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new MoveTowerArm(3, 150),

            // TODO Extend the arm

            // TODO open grabber

            new DriveDistance(-8,150),

            // TODO Retract the arm

            new MoveTowerArm(1, 150),
          
            new FinishAuto()
        );
    }       
}
