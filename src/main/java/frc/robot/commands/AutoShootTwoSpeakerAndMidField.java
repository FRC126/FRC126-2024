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
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
//import frc.robot.RobotMap;

public class AutoShootTwoSpeakerAndMidField extends SequentialCommandGroup {
    public AutoShootTwoSpeakerAndMidField(Robot.targetTypes targetType) {
        Robot.targetType=targetType;

        int direction = Robot.getDirection(targetType);

        addCommands(
            new InstantCommand(Robot.swerveDrive::resetEncoders, Robot.swerveDrive),          
            new InstantCommand(Robot.swerveDrive::brakesOn, Robot.swerveDrive),

            new AutoFirstNote(),
            new AutoSecondNote(),            

            new DriveWork(.4,(.3*direction),0,40,150),
            new DriveWork(.4,0,0,40,150),

            new FinishAuto()
        );    
    }
}
