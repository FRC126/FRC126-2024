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

public class AutoThirdNote extends SequentialCommandGroup {
    public AutoThirdNote(Robot.targetTypes targetType) {

        int direction = Robot.getDirection(targetType);

        addCommands(         
            new InstantCommand(Robot.swerveDrive::resetEncoders, Robot.swerveDrive),
            new InstantCommand(Robot.swerveDrive::brakesOn, Robot.swerveDrive),

            new DriveWork(-0.5,(.25*direction),0,18,100),     

            new ParallelCommandGroup(
                // Run the Pickup
                new PickupWork(125, false),
                new DriveWork(0.5,(.25*direction),0,26,125)
            ),    
            
            new ParallelCommandGroup(
                new DriveWork(-.5,(-.65*direction),0,18,100)
            ),

            new ThrowerWork(RobotMap.throwerSpeed, 75)
        );        
    }
}
