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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.targetTypes;

public class TargetAimWork extends Command {
    int iters, speed;
    double angle;
    int throwingIters=0;
    int reachedOne=0, reachedTwo=0;

    /**********************************************************************************
     **********************************************************************************/

    public TargetAimWork(int iters) {
        addRequirements(Robot.lidar);
        this.iters = iters;
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
        Robot.targetType = targetTypes.TargetOne;

        Robot.limeLight.trackTarget();
        SmartDashboard.putBoolean("ShootNow", Robot.shootNow);        
    }

    /**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
     **********************************************************************************/

    @Override
    public boolean isFinished() {
        iters--;

        if (iters == 0) {
            Robot.thrower.cancel();
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
    }
}
