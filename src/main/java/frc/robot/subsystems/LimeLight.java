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
import frc.robot.Robot.targetTypes;
import frc.robot.commands.*;
import frc.robot.util.Smoother;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
    double pipelineLast=0;

    static int itersToCapture = 4;

    private Smoother taSmoother = new Smoother(itersToCapture);
    private Smoother txSmoother = new Smoother(itersToCapture);
    private Smoother tySmoother = new Smoother(itersToCapture);

	/************************************************************************
	 ************************************************************************/

    public LimeLight() {
        // Register this subsystem with command scheduler and set the default command
        super();
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

     public void setActiveSeek(boolean seek) {
        activeSeek = seek;
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
        if (Robot.targetType == targetTypes.NoTarget) {
            return;
        }
        int pipeline=Robot.targetType.getPipeline();

        if (pipeline != pipelineLast) {
            validCount=0;
            setllTargetData(false, 0, 0, 0);
        }
        pipelineLast=pipeline;

        LimelightHelpers.setPipelineIndex(null, pipeline);
        LimelightHelpers.setCameraMode_Processor(null);
        LimelightHelpers.setLEDMode_PipelineControl(null);

        SmartDashboard.putNumber("Limelight Pipe", LimelightHelpers.getCurrentPipelineIndex(null));

        double tx = txSmoother.sampleAndGetAverage(LimelightHelpers.getTX(null));
        double ty = tySmoother.sampleAndGetAverage(LimelightHelpers.getTY(null));
        double ta = taSmoother.sampleAndGetAverage(LimelightHelpers.getTA(null));        
        boolean tv = LimelightHelpers.getTV(null);
        
        if (limeLightDebug) {
            //post to smart dashboard periodically
            SmartDashboard.putNumber("LimelightX", tx);
            SmartDashboard.putNumber("LimelightY", ty);
            SmartDashboard.putNumber("LimelightArea", ta);
            SmartDashboard.putBoolean("LimelightValid", tv);
        }    

        if (tv) {
            setllTargetData(true, ta, tx, ty);
        } else {
            setllTargetData(false, 0, 0, 0);
        }        
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
                double llTargetX = Robot.limeLight.getllTargetX()-10;
                if ( llTargetX < -1.5 || llTargetX > 1.5) {
                    if ( activeSeek && Robot.doAutoCommand() ) {
                        Robot.autoMove=true;
                        Robot.autoCommand=new AutoTurn(llTargetX,25);
                        Robot.autoCommand.schedule();
                    }	   
                    angleOffset=llTargetX;
                    centeredCount=0;
                    Robot.shootNow=false;

                    // .41 Area 48 degress 3000rpm
                    // .32 Area 43 dgreees 3000rpm
                    // .24 area 38 degrees 3000rpm
                    // .18 area 34.5 degrees 3000rpm
                    // .137 area 32.5 degrees 3000rpm
                    // .9 area 27.6 dgrress 3300

                    double area = Robot.limeLight.getllTargetArea();

                    if ( activeSeek ) {
                        double angle= 50 - ((45 - (area*100)) *.675);

                        if (angle < 20 || angle>65) { angle=30; }
            SmartDashboard.putNumber("Limelight throwerangle", angle);

                        Robot.autoMoveThrower=true;
                        Robot.thrower.setThrowerPosition(angle);
                    } else {
                        Robot.autoMoveThrower=false;    
                    }    
                } else {
                    double area = Robot.limeLight.getllTargetArea();

                    if ( activeSeek ) {
                        double angle=50 - ((45 - (area*100)) *.675);

                        if (angle < 20 || angle>65) { angle=30; }
            SmartDashboard.putNumber("Limelight throwerangle", angle);

                        Robot.autoMoveThrower=true;
                        if (Robot.thrower.setThrowerPosition(angle)) {
                            Robot.shootNow=true;
                        }        
                    } else {
                        Robot.autoMoveThrower=false;    
                    }    


                    // Target is centered, don't turn the robot
                    angleOffset=0;
                    centeredCount++;
                    if (centeredCount > 5) {

                        //if (activeSeek && Robot.doAutoCommand()) {
                        //    Robot.autoMove=true;
                        //    // TODO Need to calculate the angle and rpm
                        //    Robot.autoCommand=new AutoThrow(3500,45);
                        //    Robot.autoCommand.schedule();
                        //	   
                        //Robot.shootNow=true;

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
                Robot.autoMoveThrower=false;
            }    
        }
    }          

}

