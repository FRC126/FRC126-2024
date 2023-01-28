/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2023 Code       
	Go get em gaels!

***********************************/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**********************************************************************************
 **********************************************************************************/

 public class TurnDegrees extends CommandBase {
    double driveFb;
    double driveLr;
    double startAngle;
    double degrees;
    int iters;

	/**********************************************************************************
	 **********************************************************************************/
	
    public TurnDegrees(double lr, double degrees_in, int iters_in ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        driveFb = 0;
        driveLr = lr;
        degrees = degrees_in;
        iters = iters_in;
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/
	
    public void initialize() {
        // Save the starting angle for the turn
        startAngle = Robot.internalData.getGyroAngle();
        Robot.driveBase.resetEncoders();
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
    public void execute() {
        // get the current angle from the gyro
        double currentDegrees = Robot.internalData.getGyroAngle();

        if (currentDegrees <= startAngle + degrees - 5) {
            // We still need to turn more to reach our target angle
            Robot.driveBase.Drive(driveFb, driveLr);
        } else {
            // Stop turing
            Robot.driveBase.Drive(0, 0);
        }
     }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    public boolean isFinished() {
        iters--;
        double currentDegrees = Robot.internalData.getGyroAngle();

        if (currentDegrees >= startAngle + degrees -5 || iters <= 0) {
            // We have reached our target angle or run out of time to do so.
            Robot.driveBase.Drive(0, 0);
            return true;
        }
        return false;
    }

	/**********************************************************************************
     * Called once after isFinished returns true
	 **********************************************************************************/
	
    public void end(boolean isInteruppted) {
        Robot.driveBase.Drive(0, 0);
    }
}

