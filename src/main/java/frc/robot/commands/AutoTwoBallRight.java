/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2022 Code       
	Go get em gaels!

***********************************/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.Robot;
 
/**********************************************************************************
 **********************************************************************************/

public class AutoTwoBallRight extends SequentialCommandGroup {
    public AutoTwoBallRight() {
        // TODO Target RPM for throw after picking up second ball
        //int throwRPM=14000;

        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            new ParallelCommandGroup(
                // Shift the Transmission to Low
                // new InstantCommand(Robot.driveBase::shiftDown, Robot.driveBase),

                // Extend the Intake
                // new InstantCommand(Robot.ballIntake::ExtendIntake, Robot.ballIntake),

                //Backup to throw the ball
                new DriveWork(-0.4, 0, 45)

                // Throw the Ball
                // new ThrowerWork(throwRPM, 0, true, false)
            ),    

      
            new ParallelCommandGroup(
                // Stop the trower
                // new InstantCommand(Robot.ballThrower::ThrowerIntakeStop, Robot.ballThrower),

                // Turn by degrees
                new TurnDegrees(-0.45, 160, 150)
                ),    

            new ParallelCommandGroup(
                // Start Running the Intake
                // new InstantCommand(Robot.ballIntake::IntakeRun, Robot.ballIntake),

                // Drive to the Ball
                new DriveWork(.40, 0, 60)
            ),    


            new ParallelCommandGroup(
                // Keep running intake for a little bit, will stop when done
                // Turn by degrees
                new TurnDegrees(-0.45, 135, 150)
            ),

            //new InstantCommand(Robot.ballIntake::IntakeStop, Robot.ballIntake),

            new ParallelCommandGroup(
                // new IntakeWork(true, 50),

                // Drive forward to the target
                new DriveWork(.45, 0, 40)
            )

            // Throw the ball
            //new ThrowerWorkStop(throwRPM+200, 0, true),
            
            // Stop the trower
            //new InstantCommand(Robot.ballThrower::ThrowerIntakeStop, Robot.ballThrower),
            //new ThrowerWork(0, 0, false, true)
        );
    }       
}
