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

public class AutoMoveRight extends SequentialCommandGroup {
    public AutoMoveRight() {
        /**********************************************************************************
         **********************************************************************************/

        double angle=30;

        addCommands(
            new DriveDistance(-12,150),

            new TurnDegreesBetter(angle, 150),

            new DriveDistance(6,150),

            new TurnDegreesBetter((angle * -1), 150),

            new DriveDistance(7, 150),
            
            new FinishAuto()
        );
    }   
    
    
}
