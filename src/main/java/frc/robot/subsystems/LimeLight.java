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
        /*
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

        SmartDashboard.putNumber("FOO TV", tv);
        SmartDashboard.putNumber("FOO Tx", tx);
        SmartDashboard.putNumber("FOO Ty", ty);
        SmartDashboard.putNumber("FOO Ta", ta);
*/
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry nttx = table.getEntry("tx");
        NetworkTableEntry ntty = table.getEntry("ty");
        NetworkTableEntry ntta = table.getEntry("ta");
        NetworkTableEntry nttv = table.getEntry("tv");
        
        //read values periodically
        double tx = nttx.getDouble(0.0);
        double ty = ntty.getDouble(0.0);
        double ta = ntta.getDouble(0.0);
        double tv = nttv.getDouble(0.0);
        
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
        SmartDashboard.putNumber("LimelightValid", tv);

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
            if ( foo < -1.5 || foo > 1.5) {
                if ( Robot.doAutoCommand() ) {
                    double currentAngle = Robot.navxMXP.getAngle();
                    SmartDashboard.putNumber("shootAngle", currentAngle);
                    SmartDashboard.putNumber("fooAngle", foo);
                    SmartDashboard.putNumber("shootAngle", currentAngle);
                    Robot.autoMove=true;
                    Robot.autoCommand=new AutoTurn(foo,500);
                    Robot.autoCommand.schedule();
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

