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

public class AutoMoveRight extends SequentialCommandGroup {
    public AutoMoveRight(int multiplier) {
        /**********************************************************************************
         **********************************************************************************/

        double angle=20;

        addCommands(
            new DriveDistance(-6*multiplier,150),

            new TurnDegreesBetter(angle, 150),

            new DriveDistance(3*multiplier,150),

            new TurnDegreesBetter((angle * -1), 150),

            new DriveDistance(3*multiplier+1, 150),
            
            new FinishAuto()
        );
    }   
    
    
}
