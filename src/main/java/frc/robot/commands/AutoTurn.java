/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2024 Code       
	Go get em gaels!

***********************************/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**********************************************************************************
 **********************************************************************************/

public class AutoTurn extends SequentialCommandGroup {
    public AutoTurn(double angle, int iters) {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new TurnDegreesWork(angle, iters),
            new FinishAuto()
        );
    }       
}
