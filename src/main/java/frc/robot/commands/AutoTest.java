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

public class AutoTest extends SequentialCommandGroup {
    public AutoTest() {
        /**********************************************************************************
         **********************************************************************************/
        
        addCommands(
            new DriveWork(.2,0,0,12,100),
            new TurnDegreesBetter(90,100),
            new DriveWork(.2,0,0,12,100),
            new TurnDegreesBetter(90,100),
            new DriveWork(.2,0,0,12,100),
            new TurnDegreesBetter(90,100),
            new DriveWork(.2,0,0,12,100),
            new TurnDegreesBetter(90,100),
            new FinishAuto()
        );
    }
}
