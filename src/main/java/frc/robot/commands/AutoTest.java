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

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

/**********************************************************************************
 **********************************************************************************/

public class AutoTest extends SequentialCommandGroup {
    public AutoTest() {
        /**********************************************************************************
         **********************************************************************************/
        
        addCommands(
            new InstantCommand(Robot.swerveDrive::resetEncoders, Robot.swerveDrive),

            /*
            new DriveWork(.2,0,0,12,100),
            new TurnDegreesBetter(90,100),
            new DriveWork(.2,0,0,12,100),
            new TurnDegreesBetter(90,100),
            new DriveWork(.2,0,0,12,100),
            new TurnDegreesBetter(90,100),
            new DriveWork(.2,0,0,12,100),
            new TurnDegreesBetter(90,100),
            */

            /*
            new DriveWork(.3,.3,0,12,250),
            new DriveWork(.3,-.3,0,12,250),
            new DriveWork(-.3,-.3,0,12,250),
            new DriveWork(-.3,.3,0,12,250),
            */

            // Aim at speaker
            // Throw Note in Speaker 
            
            // Run Pickup
            new DriveWork(-0.3,0,0,18,250),
            new TurnDegreesBetter(-20,250),
            // Aim at speaker
            // Throw note in speaker
            new TurnDegreesBetter(-70,250),
            //Run pickup
            new DriveWork(0,0.3,0,24,250),
            new TurnDegreesBetter(60,250),
            // Aim at speaker
            // Throw note in speaker


            new FinishAuto()
        );
    }
}
