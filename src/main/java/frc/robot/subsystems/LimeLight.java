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

package frc.robot.subsystems;

import frc.robot.Robot;
//import frc.robot.Robot;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LimeLight extends SubsystemBase {

    private boolean llTargetValid;
    private double llTargetArea;
    private double llTargetX;
    private double llTargetY;
    private double turretTarget;
    private int validCount;
    private int missedCount;
    private int centeredCount;
    private int iter;
    public static SequentialCommandGroup throwCommand;


	/************************************************************************
	 ************************************************************************/

    public LimeLight() {
        // Register this subsystem with command scheduler and set the default command
        CommandScheduler.getInstance().registerSubsystem(this);
        setDefaultCommand(new LimeLightWork(this));

        llTargetValid=false;
        llTargetArea = 0.0;
        llTargetX = 0.0;
        llTargetY = 0.0;
        turretTarget = 0;
        
        validCount=0;
        missedCount=0;

        centeredCount=0;

        iter=0;

        Robot.isThrowCommand = false;

    }

	/************************************************************************
	 ************************************************************************/
    @Override
    public void periodic() {
        //if ( Robot.trackTarget == Robot.targetTypes.throwingTarget ||
        //Robot.trackTarget == Robot.targetTypes.turretOnly ) {
            getEntry("pipeline").setNumber(0);
        //} else {
        //    getEntry("pipeline").setNumber(1);
        //}
        double tv = getEntry("tv").getDouble(0);
        double tx = getEntry("tx").getDouble(0);
        double ty = getEntry("ty").getDouble(0);
        double ta = getEntry("ta").getDouble(0);
        
        if (tv < 1.0) {
            setllTargetData(false, 0, 0, 0);
        } else {
            setllTargetData(true, ta, tx, ty);
        }        
    }

	/************************************************************************
	 ************************************************************************/

	public boolean getllTargetValid() {
       return llTargetValid;
    }   

	/************************************************************************
	 ************************************************************************/

	public double getllTargetArea() {
        return llTargetArea;
    }   
 
	/************************************************************************
	 ************************************************************************/

    public double getllTargetX() {
        return llTargetX;
    }   

	/************************************************************************
	 ************************************************************************/

    public double getllTargetY() {
        return llTargetY;
    }   

	/************************************************************************
	 ************************************************************************/

    public double getllTurretTarget() {
        return turretTarget;
     }   

	/************************************************************************
	 ************************************************************************/

    public void setllTurretTarget(double target) {
        turretTarget = target;
    }   

	/************************************************************************
	 ************************************************************************/

    public void setllTargetData(boolean isValid,
                                double targetArea,
                                double targetX,
                                double targetY) {
        llTargetValid = isValid;
        llTargetArea = targetArea;
        llTargetX = targetX;
        llTargetY = targetY;
    }    

	/************************************************************************
	 ************************************************************************/

     public void getCameraData() {

        //if ( Robot.trackTarget == Robot.targetTypes.throwingTarget ||
        //Robot.trackTarget == Robot.targetTypes.turretOnly ) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        //} else {
        //    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
        //}
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        
        if (tv < 1.0) {
            setllTargetData(false, 0, 0, 0);
        } else {
            setllTargetData(true, ta, tx, ty);
        }        
    }

    /************************************************************************
	 ************************************************************************/

    public void setLED(boolean onOff) {
        getEntry("ledMode").setNumber(onOff ? 1 : 0);
    }

	/************************************************************************
	 ************************************************************************/

    public void setCameraMode(boolean vision) {
        getEntry("camMode").setNumber(vision ? 0 : 1);
    }

	/************************************************************************
	 ************************************************************************/

    public void setPipeline(int pipeline) {
        getEntry("pipeline").setNumber(pipeline);
    }

	/************************************************************************
	 ************************************************************************/

    public void setStreamMode(int mode) {
        getEntry("stream").setNumber(mode);
    }

    /************************************************************************
	 ************************************************************************/

    public NetworkTableEntry getEntry(String entry) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry);   
    }

    private void dashboardData() {
        SmartDashboard.putBoolean("LL Valid", Robot.limeLight.getllTargetValid());
        SmartDashboard.putNumber("LL Area", getllTargetArea());
        SmartDashboard.putNumber("LL X", getllTargetX());
        SmartDashboard.putBoolean("shootnow", Robot.shootNow);
     }

   	/************************************************************************
	 ************************************************************************/

    public void trackTarget() {

        if (Robot.targetType != Robot.targetTypes.TargetSeek) {
            // If we are not seeking a target, then reset all target 
            // data and return

            Robot.robotDrive=0;
            Robot.robotTurn=0;
            Robot.shootNow=false;

            setllTargetData(false, 0, 0, 0);
            
            validCount=0;
            missedCount=0;
            centeredCount=0;
            iter=0;

            if (Robot.isThrowCommand == true) {
                // Cancel any outstanding throw commands.
                throwCommand.cancel();
                Robot.isThrowCommand = false;
            }
            
            dashboardData();
		 	return;
        }
        
        Robot.limeLight.getCameraData();

        if (Robot.limeLight.getllTargetValid()){
            // We found a valid vision target.
            iter=0;

            // Keep track of the number of time we seen a valid target
            validCount++;
            missedCount=0;

            Robot.robotDrive=0;

            double area = Robot.limeLight.getllTargetArea();
            double threshold;

            // The further the object is awway from the us, allow less deviation from the center.
            if (area < .2) {
                threshold = 1.5;
            } else if (area < .5) {
                threshold = 2.5;
            } else if (area < 1) {
                threshold = 3.5;
            } else {
                threshold = 4.5;
            }

            if ( Robot.limeLight.getllTargetX() < ( -1 * threshold ) ) {
                // Target is to the left of the Robot, need to move left
                Robot.robotTurn=-.25;
                if ( Robot.limeLight.getllTargetX() + threshold < ( -1 * threshold ) ) {
                    Robot.robotTurn=-.35;
                }
                centeredCount=0;
                Robot.shootNow=false;
            } else if ( Robot.limeLight.getllTargetX() > threshold ) {
                // Target is to the left of the Robot, need to move right
                Robot.robotTurn=.25;
                if ( Robot.limeLight.getllTargetX() - threshold > threshold ) {
                    Robot.robotTurn=.35;
                }
                centeredCount=0;
                Robot.shootNow=false;
            } else {
                // Target is centered, don't turn the robot
                centeredCount++;
                if (centeredCount > 10) {
                    // If we have stayed centered on the target for 10 interations, 
                    // throw the ball
                    Robot.shootNow=true;
                } else {
                   Robot.shootNow=false;
                }
                Robot.robotTurn=0;
            }
        } else {
            if (validCount > 10 && missedCount <= 3) {
                // Don't change the old data, so we won't stop on dropping a frame or 3
                missedCount++;
                // Stop turning if we dropped a frame.
                Robot.robotTurn = 0;
            } else {
                iter++;

                // Don't move forward or back
                Robot.robotDrive=0;
                // Initialize all target data
                Robot.shootNow=false; 
                setllTargetData(false, 0, 0, 0);
                validCount=0;
                missedCount=0;
                centeredCount=0;
    
                if (Robot.isThrowCommand == true) {
                    throwCommand.cancel();
                    Robot.isThrowCommand = false;
                }
    
                if (iter > 10 && iter < 350) {
                    // Try turning until we pick up a target
                    Robot.robotTurn= -0.3;
                } else {
                    Robot.robotTurn=0;
                }    

                if ( iter > 500 ) { 
                    iter=0; 
                }              
            }    
        }

        if (Robot.shootNow && Robot.isThrowCommand == false) {
            // If we are centered on the target, and shooNow is true, the create
            // an autoThrow command to throw the balls.

            // TODO Need to set RPM based on targetArea.
            throwCommand = new AutoThrow(14000);
            Robot.isThrowCommand = true;
        }

        dashboardData();
    }          

}

