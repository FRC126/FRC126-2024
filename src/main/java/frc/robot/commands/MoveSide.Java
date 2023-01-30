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
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.Robot;
 
/**********************************************************************************
 **********************************************************************************/

public class MoveSide extends SequentialCommandGroup {
    public MoveSide(int direction) {
        /**********************************************************************************
         **********************************************************************************/

        double speed=0.25;
        double angle=30 * direction;

        addCommands(
            new DriveWork(speed * -1,0,50),

            new TurnDegreesBetter(angle, 150),

            new DriveWork(speed,0,25),

            new TurnDegreesBetter((angle * -1), 150),

            new DriveWork(speed,0,25)
        );
    }       
}
