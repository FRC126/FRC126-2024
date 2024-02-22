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

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LEDSubsystem;

public class TargetAimWork extends Command {
    int iters;
    Robot.targetTypes target;
    boolean aimed=false;

    /**********************************************************************************
     **********************************************************************************/

    public TargetAimWork(Robot.targetTypes target, int iters) {
        addRequirements(Robot.lidar);
        this.iters = iters;
        this.target = target;
    }

    /**********************************************************************************
     * Called just before this Command runs the first time
     **********************************************************************************/

    @Override
    public void initialize() {
    }

    /**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
     **********************************************************************************/

    @Override
    public void execute() {
        Robot.limeLight.setActiveSeek(true);
        Robot.targetType = target;

        if (Robot.targetType == Robot.targetTypes.TargetOne) {
            Robot.Leds.setMode(LEDSubsystem.LEDModes.AimingSpeaker);
        } else if (Robot.targetType == Robot.targetTypes.TargetTwo) {
            Robot.Leds.setMode(LEDSubsystem.LEDModes.AimingAmp);
        }

        Robot.limeLight.trackTarget();

        aimed = Robot.limeLight.seekTarget();
    }

    /**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
     **********************************************************************************/

    @Override
    public boolean isFinished() {
        iters--;

        if (iters == 0 || aimed) {
            return true;
        }
        return false;
    }

    /**********************************************************************************
     * Called once after isFinished returns true
     **********************************************************************************/

    @Override
    public void end(boolean isInteruppted) {
        Robot.thrower.cancel();
        Robot.autoMoveThrower=false;
        Robot.limeLight.setActiveSeek(false);
    }
}
