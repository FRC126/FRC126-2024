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
import frc.robot.RobotMap;

/**********************************************************************************
 **********************************************************************************/

public class AutoCatchCone extends SequentialCommandGroup {
    public AutoCatchCone() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new MoveGrabber(RobotMap.grabberCubePos-50, 250),
            
            new MoveTowerArm(RobotMap.towerArmRetractedPos+5, 250),

            new FinishAuto()
       );
    }       
}
