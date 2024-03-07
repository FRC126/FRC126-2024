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

public class AutoShootSpeakerAndOneMore extends SequentialCommandGroup {
    public AutoShootSpeakerAndOneMore(Robot.targetTypes targetType) {
        // Move thrower arm to specific position
        // Eject the note

        Robot.targetType=targetType;

        addCommands(
            new InstantCommand(Robot.swerveDrive::resetEncoders, Robot.swerveDrive),

            new ParallelCommandGroup(
                new ThrowerAngle(RobotMap.throwerCloseAngle,150),
                // Throw the Note
                new ThrowerWork(RobotMap.throwerSpeed, 150)
            ),    

            new ParallelCommandGroup(
                // Run the Pickup
                new PickupWork(150, false),
                // Drive over the next note
                new DriveWork(.35,0,0,32,150)
            ),
            new DriveWork(-0.30,0,0,12,150),
        
            new ThrowerAngle(43.9,150),

            // TODO Aim at speaker
            //new TargetAimWork(targetType, 150,false),
            // Throw note in speaker
            new ThrowerWork(RobotMap.throwerSpeed, 150),            
            
            new FinishAuto()
        );
    }
}
