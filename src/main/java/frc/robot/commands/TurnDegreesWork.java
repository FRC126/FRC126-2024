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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

 public class TurnDegreesWork extends Command {
    double startAngle;
    double targetDegrees;
    int iters;
    static private double driftAllowance=1;
    int targetReached=0;

	/**********************************************************************************
	 **********************************************************************************/
	
    public TurnDegreesWork(double degrees_in, int iters_in ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        targetDegrees = degrees_in;
        iters = iters_in;
        targetReached=0;
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/
	
     @Override
     public void initialize() {
        // Save the starting angle for the turn
        //Robot.internalData.resetGyro();
        //startAngle=Robot.internalData.getGyroAngle();
        
        double currentDegrees = Robot.navxMXP.getAngle();      
        startAngle = currentDegrees;
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
    @Override
    public void execute() {
        double driveRotate=0;

        // get the current angle from the gyro
        double currentDegrees = Robot.navxMXP.getAngle();      
        double target = startAngle + targetDegrees;
        double diff = Math.abs(target) - Math.abs(currentDegrees);

        double tmp = diff / 250;
        tmp = Robot.boundSpeed(tmp, .25, .01 );

        if (Math.abs(diff) < driftAllowance) {
            // We are at the right angle
            targetReached++;
            driveRotate=0;
            Robot.swerveDrive.brakesOn();
        } else if (currentDegrees < target) {
            driveRotate=tmp;
            targetReached=0;
        } else {
            driveRotate=tmp*-1;
            targetReached=0;
        }

        SmartDashboard.putNumber("Turn Current Degrees",currentDegrees);
        SmartDashboard.putNumber("Turn Target Degrees",target);
        SmartDashboard.putNumber("Turn diff",diff);
        SmartDashboard.putNumber("Turn Target Reached",targetReached);

        Robot.swerveDrive.Drive(0, 0, driveRotate);
    }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    @Override
    public boolean isFinished() {
        iters--;

        if (targetReached > 3 || iters <= 0 || !Robot.checkAutoCommand()) {
            // We have reached our target angle or run out of time to do so.
            Robot.swerveDrive.brakesOff();
            Robot.swerveDrive.Drive(0, 0, 0);
            return true;
        }

        return false;
    }

	/**********************************************************************************
     * Called once after isFinished returns true
	 **********************************************************************************/
	
    @Override
    public void end(boolean isInteruppted) {
        Robot.swerveDrive.brakesOff();
        Robot.swerveDrive.Drive(0, 0, 0);
    }
}