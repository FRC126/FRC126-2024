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

public class PickupWork extends Command {

    int iters;

    /**********************************************************************************
     **********************************************************************************/

    public PickupWork(int itersIn) {
        iters = itersIn;
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
		//double pickupMotorSpeed = SmartDashboard.getNumber(Robot.PICKUP_MOTOR_SPEED_STRING, 0.0);
        //Robot.pickup.runMotor(pickupMotorSpeed);
        Robot.pickup.runMotor(0.7);
        //Robot.thrower.throwerTriggerOn();
    }

    /**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
     **********************************************************************************/

    @Override
    public boolean isFinished() {
        iters--;
        boolean haveNote=false;

        haveNote=Robot.thrower.getPhotoSensor();

        if (haveNote || iters == 0 || !Robot.checkAutoCommand()) {
            Robot.pickup.cancel();
            //Robot.thrower.throwerTriggerOff();
            return true;
        }
        return false;
    }

    /**********************************************************************************
     * Called once after isFinished returns true
     **********************************************************************************/

    @Override
    public void end(boolean isInteruppted) {
        Robot.pickup.cancel();
        //Robot.thrower.throwerTriggerOff();
    }

}
