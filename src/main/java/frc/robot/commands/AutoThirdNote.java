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

            new TurnDegreesWorkFixed(83 * direction, 100),

            new InstantCommand(Robot.swerveDrive::resetYaw, Robot.swerveDrive),

            new ParallelCommandGroup(
                new DriveWork(.4,0,0,30,125),     
                // Run the Pickup
                new PickupWork(125, false)
            ),    

            new ParallelCommandGroup(
                new TurnDegreesWorkFixed(-44 * direction, 100),
                new ThrowerAngle(33,150)
            ),


            new ThrowerWork(RobotMap.throwerSpeed, 75),

            new TurnDegreesWorkFixed(120 * direction, 150)

        );        
    }
}
