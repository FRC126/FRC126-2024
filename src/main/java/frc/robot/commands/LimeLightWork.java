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

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**********************************************************************************
 **********************************************************************************/

public class LimeLightWork extends Command {
    public static int iter=0;
    JoystickWrapper driveJoystick;
    private List<String> limelightParams = Arrays.asList("tv", "tx", "ty", "ta", "ts", "tl", "tshort", "tlong", "thor", "tvert", "getpipe", "camtran");

	/************************************************************************
	 ************************************************************************/

    public LimeLightWork(LimeLight subsystem) {
		addRequirements(subsystem);
        driveJoystick = new JoystickWrapper(Robot.oi.driveController, 0.05);
    }

	/************************************************************************
     * Called just before this Command runs the first time
	 ************************************************************************/

    public void initialize() {
    }

	/************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 ************************************************************************/

    public void execute() {

        //limelightParams.forEach(e -> {
        //    SmartDashboard.putNumber("LL " + e, Robot.limeLight.getEntry(e).getDouble(0));
        //});
        
        if (driveJoystick.getPovUp()) {
            Robot.targetType = Robot.targetTypes.TargetSeek;
        } else {
            Robot.targetType = Robot.targetTypes.NoTarget;
        }     
        
        Robot.limeLight.trackTarget();
    }

	/************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 ************************************************************************/

    public boolean isFinished() {
        return false;
    }
}
