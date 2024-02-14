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

    private boolean activeSeek=false;
    private boolean llTargetValid;
    private double llTargetArea;
    private double llTargetX;
    private double llTargetY;
    private double turretTarget;
    private int validCount;
    private int missedCount;
    private int centeredCount;
    private double angleOffset;

    public static SequentialCommandGroup throwCommand;
    boolean limeLightDebug=true;
    NetworkTable table;
    double pipelineLast=0;

	/************************************************************************
	 ************************************************************************/

    public LimeLight() {
        // Register this subsystem with command scheduler and set the default command
        CommandScheduler.getInstance().registerSubsystem(this);
        setDefaultCommand(new LimeLightControl(this));

        llTargetValid=false;
        llTargetArea = 0.0;
        llTargetX = 0.0;
        llTargetY = 0.0;
        turretTarget = 0;
        angleOffset=0;
        
        validCount=0;
        missedCount=0;

        centeredCount=0;

        table = NetworkTableInstance.getDefault().getTable("limelight");

    }

	/************************************************************************
	 ************************************************************************/
    @Override
    public void periodic() {
    }

	/************************************************************************
	 ************************************************************************/

	public double getAngleOffset() {
       return angleOffset;
    }   

    /************************************************************************
	 ************************************************************************/

	public boolean getActiveSeek() {
       return activeSeek;
    }   

   	/************************************************************************
	 ************************************************************************/

     public boolean setActiveSeek(boolean seek) {
        activeSeek = seek;
        return activeSeek;
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
        double pipeline=0;

        // Set Pipeline
        switch (Robot.targetType) {
            case NoTarget: { return; }
            case TargetSeek: { pipeline=0; break; }
            case TargetOne: { pipeline=1; break; }
            case TargetTwo: { pipeline=2; break; }
            case TargetThree: { pipeline=3; break; }
            case TargetFour: { pipeline=4; break; }
        }

        if (pipeline != pipelineLast) {
            validCount=0;
            setllTargetData(false, 0, 0, 0);
        }
        pipelineLast=pipeline;

        setPipeline(pipeline);
        setCameraMode(true);
        setLED(false);

        int foo=getPipeline();
        SmartDashboard.putNumber("Limelight Pipe", foo);

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

        if (tv < .9) {
            setllTargetData(false, 0, 0, 0);
        } else {
            setllTargetData(true, ta, tx, ty);
        }        
    }

    /************************************************************************
	 ************************************************************************/

    public void setPipeline(double pipelineIn) {
        double pipeline=pipelineIn;
        if (pipeline>9) { pipeline=9; }
        if (pipeline<0) { pipeline=0; }
        table.getEntry("pipeline").setValue(pipeline);
    }

    /************************************************************************
	 ************************************************************************/

    public int getPipeline() {
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        int pipe = (int)pipeline.getDouble(0.0);
        return(pipe);
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

    public void setStreamMode(int mode) {
        getEntry("stream").setNumber(mode);
    }

    /************************************************************************
	 ************************************************************************/

    public NetworkTableEntry getEntry(String entry) {
        return table.getEntry(entry);   
    }

   	/************************************************************************
	 ************************************************************************/

    public void trackTarget() {
        if (Robot.targetType == Robot.targetTypes.NoTarget) {
            // If we are not seeking a target, then reset all target 
            // data and return
            Robot.shootNow=false;

            setllTargetData(false, 0, 0, 0);
            
            validCount=0;
            missedCount=0;
            centeredCount=0;
           
		 	return;
        }
        
        Robot.limeLight.getCameraData();

        if (Robot.limeLight.getllTargetValid()){
            // We found a valid vision target.

            // Keep track of the number of time we seen a valid target
            validCount++;
            missedCount=0;

            if (validCount > 3) {
                double foo = Robot.limeLight.getllTargetX();
                if ( foo < -1.5 || foo > 1.5) {
                    if ( activeSeek && Robot.doAutoCommand() ) {
                        Robot.autoMove=true;
                        Robot.autoCommand=new AutoTurn(foo,200);
                        Robot.autoCommand.schedule();
                    }	   
                    angleOffset=foo;
                    centeredCount=0;
                    Robot.shootNow=false;
                } else {
                    // Target is centered, don't turn the robot
                    angleOffset=0;
                    centeredCount++;
                    if (centeredCount > 5) {
                        if (activeSeek && Robot.doAutoCommand()) {
                            Robot.autoMove=true;
                            // TODO Need to calculate the angle and rpm
                            Robot.autoCommand=new AutoThrow(2500,45);
                            Robot.autoCommand.schedule();
                        }	   
                        Robot.shootNow=true;

                    } else {
                    Robot.shootNow=false;
                    }
                }
            }    
            SmartDashboard.putNumber("Limelight Centered", centeredCount);
        } else {
            if ( missedCount < 10 ) {
                // Don't change the old data, so we won't stop on dropping a frame or 3
                missedCount++;
            } else {
                // Initialize all target data
                Robot.shootNow=false; 
                setllTargetData(false, 0, 0, 0);
                validCount=0;
                missedCount=0;
                centeredCount=0;
                angleOffset=0;
            }    
        }
    }          

}

