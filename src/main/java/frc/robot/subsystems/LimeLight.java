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

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight extends SubsystemBase {

    private boolean llTargetValid;
    private double llTargetArea;
    private double llTargetX;
    private double llTargetY;
    private double turretTarget;
    private int validCount;
    private int missedCount;
    private int centeredCount;
    public static SequentialCommandGroup throwCommand;
    boolean limeLightDebug=true;


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
    }

	/************************************************************************
	 ************************************************************************/
    @Override
    public void periodic() {
        getEntry("pipeline").setNumber(0);

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

        //read values periodically
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        // Set Pipeline
        table.getEntry("pipeline").setValue(0);

        NetworkTableEntry nttx = table.getEntry("tx");
        NetworkTableEntry ntty = table.getEntry("ty");
        NetworkTableEntry ntta = table.getEntry("ta");
        NetworkTableEntry nttv = table.getEntry("tv");
        
        double tx = nttx.getDouble(0.0);
        double ty = ntty.getDouble(0.0);
        double ta = ntta.getDouble(0.0);
        double tv = nttv.getDouble(0.0);
        
        if (limeLightDebug) {
            //post to smart dashboard periodically
            SmartDashboard.putNumber("LimelightX", tx);
            SmartDashboard.putNumber("LimelightY", ty);
            SmartDashboard.putNumber("LimelightArea", ta);
            SmartDashboard.putNumber("LimelightValid", tv);
        }    

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

   	/************************************************************************
	 ************************************************************************/

     private void dashboardData() {
        if (limeLightDebug) {
            //SmartDashboard.putBoolean("LL Valid", Robot.limeLight.getllTargetValid());
            //SmartDashboard.putNumber("LL Area", getllTargetArea());
            //SmartDashboard.putNumber("LL X", getllTargetX());
            //SmartDashboard.putBoolean("shootnow", Robot.shootNow);
        }    
     }

   	/************************************************************************
	 ************************************************************************/

    public void trackTarget() {

        if (Robot.targetType != Robot.targetTypes.TargetSeek) {
            // If we are not seeking a target, then reset all target 
            // data and return
            Robot.shootNow=false;

            setllTargetData(false, 0, 0, 0);
            
            validCount=0;
            missedCount=0;
            centeredCount=0;
           
            dashboardData();
		 	return;
        }
        
        Robot.limeLight.getCameraData();

        if (Robot.limeLight.getllTargetValid()){
            // We found a valid vision target.

            // Keep track of the number of time we seen a valid target
            validCount++;
            missedCount=0;

            double foo = Robot.limeLight.getllTargetX();
            if ( foo < -1.0 || foo > 1.0) {
                if ( Robot.doAutoCommand() ) {
                    Robot.autoMove=true;
                    Robot.autoCommand=new AutoTurn(foo,500);
                    Robot.autoCommand.schedule();
                }	   
                
                centeredCount=0;
                Robot.shootNow=false;
            } else {
                // Target is centered, don't turn the robot
                centeredCount++;
                if (centeredCount > 4) {
                    // If we have stayed centered on the target for 10 interations, 
                    // drive forwards toward the target
                    if (getllTargetArea() < .5) {
                        if ( Robot.doAutoCommand() ) {
                            Robot.autoMove=true;
                            Robot.autoCommand=new AutoDrive(.1,0,0,3,50);
                            Robot.autoCommand.schedule();
                        }
                    }    	   
                    // TODO - set thrower angle based on the distance from the target

                } else {
                   Robot.shootNow=false;
                }
            }
        } else {
            if (validCount > 10 && missedCount <= 3) {
                // Don't change the old data, so we won't stop on dropping a frame or 3
                missedCount++;
            } else {
                // Initialize all target data
                Robot.shootNow=false; 
                setllTargetData(false, 0, 0, 0);
                validCount=0;
                missedCount=0;
                centeredCount=0;
            }    
        }

        dashboardData();
    }          

}

