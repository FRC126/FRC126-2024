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

public class AutoShootSpeakerAndOneMore extends SequentialCommandGroup {
    public AutoShootSpeakerAndOneMore(Robot.targetTypes targetType) {
        // Move thrower arm to specific position
        // Eject the note

        Robot.targetType=targetType;

        addCommands(
            new InstantCommand(Robot.swerveDrive::resetEncoders, Robot.swerveDrive),
                    
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
            new TargetAimWork(targetType, 250),

            // Throw note in speaker
            new ThrowerWork(3000, 150),            
            
            new FinishAuto()
        );
    }
}
