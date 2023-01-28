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
import frc.robot.Robot;
 
/**********************************************************************************
 **********************************************************************************/

public class AutoOneBall extends SequentialCommandGroup {
    public AutoOneBall() {
        // TODO Target RPM for throw after picking up second ball
        //int throwRPM=14000;

    /**********************************************************************************
     **********************************************************************************/

     addCommands(

            new ParallelCommandGroup(
                // Extend the Intake
                // new InstantCommand(Robot.ballIntake::ExtendIntake, Robot.ballIntake),

                //Backup to throw the ball
                new DriveWork(-0.3, 0, 85)
            ),    
 
            // new ThrowerWork(throwRPM, 0, true, true),
            
            // new InstantCommand(Robot.ballThrower::ThrowerIntakeStop, Robot.ballThrower),
            // new ThrowerWork(0, 0, false, true),

            // new InstantCommand(Robot.ballIntake::RetractIntake, Robot.ballIntake),

            // Backup past the line
            new DriveWork(-0.3, 0, 25)

            // Turn by degrees
            // new TurnDegrees(-0.5, 140, 150)
            );
    }       

    /******************************************************************************************
     * Called once after isFinished returns true
     ******************************************************************************************/
/*
     @Override
    public void end(boolean isInterrupted) {
        //Robot.ballIntake.IntakeStop();
        //Robot.ballIntake.RetractIntake();
        Robot.driveBase.Drive(0,0);
        //Robot.ballThrower.ThrowerIntakeStop();
        //Robot.ballThrower.throwerRPM(0);
    }  
    */
}

