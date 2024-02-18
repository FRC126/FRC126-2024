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

public class AutoPickup extends SequentialCommandGroup {
    public AutoPickup() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new PickupWork(250),
            new FinishAuto()
        );
    }       
}
