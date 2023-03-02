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

/**********************************************************************************
 **********************************************************************************/

public class AutoMoveLeft extends SequentialCommandGroup {
    public AutoMoveLeft(int multiplier) {
        /**********************************************************************************
         **********************************************************************************/

        double angle=20;

        addCommands(
            new DriveDistance(-10*multiplier,150),

            new TurnDegreesBetter(angle+5, 150),

            new DriveDistance(5*multiplier,150),

            new TurnDegreesBetter(((angle) * -1), 150),

            new DriveDistance(4*multiplier, 150),
            
            new FinishAuto()
        );
    }       
}
