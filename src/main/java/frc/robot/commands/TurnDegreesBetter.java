/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/
    Team 126 2022 Code       
	Go get em gaels!
***********************************/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

 public class TurnDegreesBetter extends CommandBase {
    double startAngle;
    double targetDegrees;
    int iters;
    static private double driftAllowance=3;
    int targetReached=0;

	/**********************************************************************************
	 **********************************************************************************/
	
    public TurnDegreesBetter(double degrees_in, int iters_in ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        targetDegrees = degrees_in;
        iters = iters_in;
        targetReached=0;
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/
	
    public void initialize() {
        // Save the starting angle for the turn
        Robot.internalData.resetGyro();
        startAngle = 0;
        //startAngle=Robot.internalData.getGyroAngle();
        Robot.driveBase.brakesOff();
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
    public void execute() {
        double driveLr=0;

        // get the current angle from the gyro
        double currentDegrees = Robot.internalData.getGyroAngle();        
        double target = startAngle + targetDegrees;
        double diff = Math.abs(target) - Math.abs(currentDegrees);

        double tmp = diff / 100;
        if ( tmp > .45) { tmp=.45; }
        if ( tmp < .20) { tmp=.20; }

        if (Math.abs(diff) < driftAllowance) {
            // We are at the right angle
            targetReached++;
            driveLr=0;
            Robot.driveBase.brakesOn();
        } else if (currentDegrees < target) {
            driveLr=tmp * -1;
            targetReached=0;
            Robot.driveBase.brakesOff();
        } else {
            driveLr=tmp;
            targetReached=0;
            Robot.driveBase.brakesOff();
        }

        SmartDashboard.putNumber("Current Degrees",currentDegrees);
        SmartDashboard.putNumber("Target Degrees",target);
        SmartDashboard.putNumber("Turn diff",diff);
        SmartDashboard.putNumber("DriveLR",driveLr);
        SmartDashboard.putNumber("TargetReached",targetReached);

        Robot.driveBase.Drive(0, driveLr);
    }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    public boolean isFinished() {
        iters--;

        if (targetReached > 5 || iters <= 0) {
            // We have reached our target angle or run out of time to do so.
            Robot.driveBase.brakesOff();
            Robot.driveBase.Drive(0, 0);
            return true;
        }

        return false;
    }

	/**********************************************************************************
     * Called once after isFinished returns true
	 **********************************************************************************/
	
    public void end(boolean isInteruppted) {
        Robot.driveBase.brakesOff();
        Robot.driveBase.Drive(0, 0);
    }
}