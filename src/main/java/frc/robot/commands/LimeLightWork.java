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

import frc.robot.Robot;
import frc.robot.subsystems.*;
import frc.robot.JoystickWrapper;
import frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/**********************************************************************************
 **********************************************************************************/

public class LimeLightWork extends Command {
    public static int iter=0;
    JoystickWrapper driveJoystick;

	/************************************************************************
	 ************************************************************************/

    public LimeLightWork(LimeLight subsystem) {
		addRequirements(subsystem);
        driveJoystick = new JoystickWrapper(Robot.oi.driveController, 0.05);
    }

	/************************************************************************
     * Called just before this Command runs the first time
	 ************************************************************************/

    @Override
    public void initialize() {
    }

	/************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 ************************************************************************/

    @Override
    public void execute() {
        if (driveJoystick.getPovUp()) {
            Robot.targetType = Robot.targetTypes.TargetSeek;
        } else if (driveJoystick.getPovLeft()) {
            Robot.targetType = Robot.targetTypes.TargetOne;
            Robot.Leds.setMode(LEDSubsystem.LEDModes.AimingSpeaker);
        } else if (driveJoystick.getPovRight()) {
            Robot.targetType = Robot.targetTypes.TargetTwo;
            Robot.Leds.setMode(LEDSubsystem.LEDModes.AimingAmp);
        } else {
            Robot.targetType = Robot.targetTypes.NoTarget;
        }     
        
        Robot.limeLight.trackTarget();
    }

	/************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 ************************************************************************/

    @Override
    public boolean isFinished() {
        return false;
    }
}
