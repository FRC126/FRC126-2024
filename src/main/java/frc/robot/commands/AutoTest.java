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
import frc.robot.Robot;

/**********************************************************************************
 **********************************************************************************/

public class AutoTest extends SequentialCommandGroup {
    public AutoTest() {
        /**********************************************************************************
         **********************************************************************************/
        Robot.swerveDrive.resetEncoders();
         addCommands(
            new ResetEncoders(),
            new DriveWork(0.2,0,0,12,100),
            new TurnDegreesBetter(90,100),
            new ResetEncoders(),
            new DriveWork(0.2,0,0,12,100),
            new TurnDegreesBetter(90,100),
            new ResetEncoders(),
            new DriveWork(0.2,0,0,12,100),
            new TurnDegreesBetter(90,100),
            new ResetEncoders(),
            new DriveWork(0.2,0,0,12,100),
            new TurnDegreesBetter(90,100),
            new ResetEncoders(),
            new FinishAuto()
        );
    }
}
