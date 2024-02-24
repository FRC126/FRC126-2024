package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoAmp extends SequentialCommandGroup {
    public AutoAmp() {
        // Move thrower arm to specific position
        // Eject the note
        addCommands(
            new ThrowerAngle(145, 250),
            new ThrowerWork(550, 250),
            new ThrowerAngle(45, 250),
            new FinishAuto()
        );
    }
}
