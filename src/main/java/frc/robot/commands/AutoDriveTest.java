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

public class AutoDriveTest extends SequentialCommandGroup {
    public AutoDriveTest() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new DriveDistance(12, 250),

            //new TurnDegreesBetter(180, 150),

            new DriveDistance(-12, 250),

            //new TurnDegreesBetter(180, 150),
            
            new FinishAuto()
        );
    }       
}
