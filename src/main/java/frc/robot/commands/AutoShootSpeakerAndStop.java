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
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class AutoShootSpeakerAndStop extends SequentialCommandGroup {
    public AutoShootSpeakerAndStop() {
        // Move thrower arm to specific position
        // Eject the note
        addCommands(
            new InstantCommand(Robot.swerveDrive::resetEncoders, Robot.swerveDrive),
          
            new ParallelCommandGroup(
                new ThrowerAngle(RobotMap.throwerCloseAngle,150),
                // Throw the Note
                new ThrowerWork(RobotMap.throwerSpeed, 150)
            ),    

            //new TurnDegreesWork(180,250),

            //new InstantCommand(Robot.swerveDrive::resetYaw, Robot.swerveDrive),
            
            new FinishAuto()
        );
    }
}
