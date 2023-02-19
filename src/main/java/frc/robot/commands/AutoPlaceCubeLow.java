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
import frc.robot.subsystems.*;

//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.Robot;
 
/**********************************************************************************
 **********************************************************************************/

public class AutoPlaceCubeLow extends SequentialCommandGroup {
    public AutoPlaceCubeLow() {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new MoveTowerArm(TowerArm.armExtendedLowPos, 250),

            new MoveGrabber(Grabber.grabberOpenPos, 250),

            new DriveDistance(-12, 150),

            new MoveGrabber(Grabber.grabberConePos, 250),

            new MoveTowerArm(TowerArm.armRetractedPos, 250),

            new FinishAuto()        
        );
    }       
}
