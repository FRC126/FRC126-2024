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

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoAmp extends SequentialCommandGroup {
    public AutoAmp() {
        // Move thrower arm to specific position
        // Eject the note
        addCommands(
            new PickupWork(25,true),
            new ThrowerAngle(145, 250),
            new ThrowerWork(600, 250),
            new ThrowerAngle(45, 250),
            new FinishAuto()
        );
    }
}
