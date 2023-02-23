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

public class AutoHighConeBalance extends SequentialCommandGroup {
    public AutoHighConeBalance() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new AutoPlaceConeHigh(),

            new TurnDegreesBetter(180,200),

            new Climb(500),

            new Balance(500),

            new FinishAuto()        
        );
    }       
}
