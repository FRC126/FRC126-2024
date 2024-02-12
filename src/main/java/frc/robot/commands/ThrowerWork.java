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
import frc.robot.subsystems.Thrower;

public class ThrowerWork extends Command {

    int iters, speed;
    double angle;
    int throwingCount=0;

    /**********************************************************************************
     **********************************************************************************/

    public ThrowerWork(int speed, double angle, int iters) {
        this.iters = iters;
        this.speed = speed;
        this.angle = angle;
        throwingCount=0;
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
        boolean reachedAngle=false;

		int reachedOne = Robot.thrower.throwerRPM(0,speed);
		int reachedTwo = Robot.thrower.throwerRPM(1,speed);

        if (throwingCount > 0) {
            Robot.thrower.throwerTriggerOn();
            throwingCount--;
        } else {
            reachedAngle = Robot.thrower.setThrowerPosition(angle);
            // If we have reached the target rpm on the thrower, run the trigger and shoot the note
            if ((reachedOne > 3 && reachedTwo > 3 && reachedAngle)) {
                Robot.thrower.throwerTriggerOn();
                throwingCount=100;
            } else {
                Robot.thrower.throwerTriggerOff();
            }
        }    
    }

    /**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
     **********************************************************************************/

    @Override
    public boolean isFinished() {
        iters--;

        if ((iters == 0 && throwingCount == 0) || throwingCount == 1 || !Robot.checkAutoCommand()) {
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
