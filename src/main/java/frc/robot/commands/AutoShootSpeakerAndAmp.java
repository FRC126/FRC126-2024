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
import frc.robot.RobotMap;

public class AutoShootSpeakerAndAmp extends SequentialCommandGroup {
    public AutoShootSpeakerAndAmp(int allianceColor) {
        // Move thrower arm to specific position
        // Eject the note

        addCommands(
            new InstantCommand(Robot.swerveDrive::resetEncoders, Robot.swerveDrive),
                    
            // Throw the Note
            new ThrowerWork(3000, 150),

            new ParallelCommandGroup(
                // Run the Pickup
                new PickupWork(200, false),
                // Drive over the next note
                new DriveWork(0.30,0,0,30,200)
            )
        );          

        if (allianceColor == Robot.redAlliance) {
            addCommands(
                new TurnDegreesWork(110,200),
                new InstantCommand(Robot.swerveDrive::resetYaw, Robot.swerveDrive),
                new DriveWork(.25, 0, 0, 18, 200) 
            );
        } else {
            addCommands(
                new TurnDegreesWork(110,200),
                new InstantCommand(Robot.swerveDrive::resetYaw, Robot.swerveDrive),
                new DriveWork(.25, 0, 0, 18, 200) 
            );
        }    

        addCommands(
            new ThrowerAngle(RobotMap.ampAngle, 250),
            new ThrowerWork(RobotMap.ampSpeed, 250),
            new ThrowerAngle(30, 250)
        );

        addCommands(
            new DriveWork(-.25, 0, 0, 12, 200), 
            new FinishAuto()
        );
    }
}
