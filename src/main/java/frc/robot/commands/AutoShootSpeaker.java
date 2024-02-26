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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class AutoShootSpeaker extends SequentialCommandGroup {
    public AutoShootSpeaker() {
        // Move thrower arm to specific position
        // Eject the note
        addCommands(
            new InstantCommand(Robot.swerveDrive::resetEncoders, Robot.swerveDrive),
          
            // TODO Aim at speaker
            new TargetAimWork(Robot.targetTypes.TargetTwo, 250),
            
            // Throw the Note
            new ThrowerWork(3000, 150),

            new ParallelCommandGroup(
                // Run the Pickup
                new PickupWork(200, false),
                // Drive over the next note
                new DriveWork(0.30,0,0,30,200)
            ),
        
            //new TurnDegreesWork(-20,250),

            // TODO Aim at speaker
            new TargetAimWork(Robot.targetTypes.TargetTwo, 250),

            // Throw note in speaker
            new ThrowerWork(3000, 150),            
            
            new FinishAuto()
        );
    }
}
