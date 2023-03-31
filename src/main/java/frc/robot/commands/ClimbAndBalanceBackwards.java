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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

 public class ClimbAndBalanceBackwards extends CommandBase {
    
    double pitch, xAxis, lastPitch, xAxisStart;
    boolean StartedClimb=false;
    boolean Balancing=false;
    boolean Holding=false;
    int iters;
    int balanceCount=0;
    int count=0;
    int startingClimbCount=0;
    static final double balanceThresholdMin=-5;
    static final double balanceThresholdMax=5;

	/**********************************************************************************
	 **********************************************************************************/
	
    public ClimbAndBalanceBackwards(int itersIn) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        iters=itersIn;
        balanceCount=0;
        pitch = Robot.navxMXP.getPitch();
        xAxisStart = Robot.navxMXP.getAngle();
    }

	/**********************************************************************************
     * Called just before this Command runs the first time
	 **********************************************************************************/
	
    public void initialize() {
        Robot.driveBase.resetEncoders();
        Robot.driveBase.brakesOn();
        StartedClimb=false;
        Balancing=false;
        Holding=false;
        balanceCount=0;
        count=0;
        startingClimbCount=0;
    }

	/**********************************************************************************
     * Called repeatedly when this Command is scheduled to run
	 **********************************************************************************/
	
    public void execute() {

        pitch = Robot.navxMXP.getPitch();
        xAxis = Robot.navxMXP.getAngle();
        double speed=0;
        double maxSpeed=.28;
        double minSpeed=.095;
        if ( !StartedClimb ) {
            // Drive Foward until we start climbing up the ramps.
            Balancing=false;
            Holding=false;
            speed=-0.30;
            if (pitch < -18) { StartedClimb = true; }
            //if (pitch < -14) { StartedClimb = true; }
            balanceCount=0;

            // We've been trying to start climbing for too long
            if ( startingClimbCount++ > 150 ) {
                speed=0;
            };

        } else if (pitch < -7 && !Holding) {
            // Robot is still pitched up, keep driving forward
            Balancing=true;
            Holding=false;
            if (lastPitch < pitch ) {
                // If the pitch is still going up, divide the pitch by 120 
               speed=Robot.boundSpeed((pitch/90), maxSpeed*-1, minSpeed*-1);
            } else {
                // If the pitch is going back down, divide the pitch by 150 
                speed=Robot.boundSpeed((pitch/110), maxSpeed*-1, minSpeed*-1);
            }  
            balanceCount=0;
        } else if (pitch > 6) {
            // Robot is pitching down, start driving backwards
            Balancing=true;
            Holding=false;
            if (lastPitch > pitch ) {
                // If the pitch is still going down, divide the pitch by 120 
                speed=Robot.boundSpeed((pitch/90), maxSpeed, minSpeed);
             } else {
                // If the pitch is going back up, divide the pitch by 150 
                speed=Robot.boundSpeed((pitch/110), maxSpeed, minSpeed);
             }  
             balanceCount=0;
        } else {
            // We are balanced, stop.
            Balancing=false;
            speed=0;
            if (pitch < 3 && pitch > -3) { balanceCount++; }
            Robot.driveBase.brakesOn();

            if (balanceCount == 1) {
                Holding=false;
            } else {
                // try to keep the robot from moving
                if (balanceCount == 3) {
                    Robot.driveBase.resetEncoders();
                    Holding=true;
                }     
                double distance = Robot.driveBase.getDistanceInches();
                if (distance > 1) {
                    speed=-0.1;
                } else if (distance < -1 ) {
                    speed=0.1;
                }
            }
        }

        lastPitch=pitch;

        SmartDashboard.putNumber("NavX Pitch",pitch);
        //SmartDashboard.putNumber("Balance Count",balanceCount);
        //SmartDashboard.putBoolean("StartedClimb",StartedClimb);
        //SmartDashboard.putBoolean("Balancing",Balancing);
        //SmartDashboard.putBoolean("Holding",Holding);
        //SmartDashboard.putNumber("Balance Speed",speed);

        Robot.driveBase.Drive(speed, 0, true, xAxisStart);
    }

	/**********************************************************************************
     * Make this return true when this Command no longer needs to run execute()
	 **********************************************************************************/
	
    public boolean isFinished() {
        iters--;
        if ( ( StartedClimb && balanceCount > 50000 ) || iters <= 0 ) {
            return true;
        }
        return false;
    }

	/**********************************************************************************
     * Called once after isFinished returns true
	 **********************************************************************************/
	
    public void end(boolean isInteruppted) {
        Robot.driveBase.cancel();
        Robot.driveBase.brakesOff();
    }
}
