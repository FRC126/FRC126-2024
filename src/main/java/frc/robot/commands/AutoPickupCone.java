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

public class AutoPickupCone extends SequentialCommandGroup {
    public AutoPickupCone() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new MoveGrabber(RobotMap.grabberOpenPos, 250),
            
            new MoveTowerArm(RobotMap.towerArmRetractedPos+8, 250),

            new MoveArmExtension(RobotMap.armExtendedPickupPos, 250),

            new MoveGrabber(RobotMap.grabberConePos, 250),

            new MoveArmExtension(RobotMap.armRetractedPos, 250),

            new MoveTowerArm(RobotMap.towerArmRetractedPos, 250),

            new FinishAuto()    
        );
    }       
}
