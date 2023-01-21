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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
 
/**********************************************************************************
 **********************************************************************************/

public class AutoThrow extends SequentialCommandGroup {
    
    public AutoThrow(int throwRPM) {
        addCommands(
            // Stop Running the Intake
            new InstantCommand(Robot.ballIntake::IntakeStop,Robot.ballIntake),

            // Spin up the thrower
            new ThrowerWork(throwRPM,0,true,true),

            new InstantCommand(Robot.ballThrower::ThrowerIntakeStop,Robot.ballThrower),
            new ThrowerWork(0,0,false,true)
        );
    }       

    /******************************************************************************************
     * Called once after isFinished returns true
     ******************************************************************************************/
    
     @Override
	public void end(boolean isInterrupted) {
        Robot.ballThrower.ThrowerIntakeStop();
        Robot.ballThrower.throwerRPM(0);
    }  
}