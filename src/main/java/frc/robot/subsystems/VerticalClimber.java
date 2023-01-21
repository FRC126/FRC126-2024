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
import frc.robot.RobotMap;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

public class VerticalClimber extends SubsystemBase {

    int limitCountLeft=0;
    int limitCountRight=0;

	/************************************************************************
	 ************************************************************************/

     public VerticalClimber() {
        // Register this subsystem with command scheduler and set the default command
        CommandScheduler.getInstance().registerSubsystem(this);
        setDefaultCommand(new ClimberControl(this));
        Robot.climberMotorLeft.setNeutralMode(NeutralMode.Brake);
        Robot.climberMotorRight.setNeutralMode(NeutralMode.Brake);
    }

	/************************************************************************
	 ************************************************************************/
    
    public void periodic() {}

	/************************************************************************
     ************************************************************************/

     public void StopClimber() {
        Robot.climberMotorLeft.set(ControlMode.PercentOutput,-0);
        Robot.climberMotorRight.set(ControlMode.PercentOutput,-0);
    }

    /************************************************************************
     ************************************************************************/

    public void checkCurrent() {
        double currentLeft=Robot.climberMotorLeft.getStatorCurrent();
        double currentRight=Robot.climberMotorRight.getStatorCurrent();

        double currentLimit = 50;

        if ( currentLeft > currentLimit ) {
            Robot.climberMotorLeft.set(ControlMode.PercentOutput,0);
            limitCountLeft=50; 
        }
        if ( currentRight > currentLimit ) {
            Robot.climberMotorRight.set(ControlMode.PercentOutput,0);
            limitCountRight=50;
        }

        SmartDashboard.putNumber("Climber Curr R", currentRight);
        SmartDashboard.putNumber("Climber Curr L", currentLeft);
        SmartDashboard.putNumber("Limit Count L", limitCountLeft);
        SmartDashboard.putNumber("Limit Count R", limitCountRight);
    }

	/************************************************************************
     ************************************************************************/

     public double getRightPos() {
        double posRight = Robot.climberMotorRight.getSelectedSensorPosition() * RobotMap.climberMotorRInversion;
        SmartDashboard.putNumber("Arm Pos Right", posRight);
        return(posRight);
     }

   	/************************************************************************
     ************************************************************************/

    public double getLeftPos() {
        double posLeft = Robot.climberMotorLeft.getSelectedSensorPosition() * RobotMap.climberMotorLInversion;
        SmartDashboard.putNumber("Arm Pos Left", posLeft);
        return(posLeft);
     }

  	/************************************************************************
     ************************************************************************/

     public void RaiseClimber() {
        //  Raise Climber
        limitCountLeft--;
        limitCountRight--;

        // TODO, need to know what the height limit is on the arm encoders.
        // TODO have position for first bar and second bar
        double heightLimit = 800000;

        // Check the current draw before we move the motors
        checkCurrent();   

        // Need to use encoder to track max extension
        double posLeft = getLeftPos();
        if (posLeft < heightLimit && limitCountLeft <= 0 ) {
            if (posLeft > heightLimit - 25000) {
                // Slow down as we get close to the limit
                Robot.climberMotorLeft.set(ControlMode.PercentOutput, 0.35 * RobotMap.climberMotorLInversion);
            } else {
                Robot.climberMotorLeft.set(ControlMode.PercentOutput, 1 * RobotMap.climberMotorLInversion);
            }    
        } else {
            Robot.climberMotorLeft.set(ControlMode.PercentOutput,-0);
        }

        double posRight = getRightPos();
        if (posRight < heightLimit && limitCountRight <= 0 ) {
            if (posRight > heightLimit - 25000) {
                // Slow down as we get close to the limit
                Robot.climberMotorRight.set(ControlMode.PercentOutput,0.35 * RobotMap.climberMotorRInversion);
            } else {
                Robot.climberMotorRight.set(ControlMode.PercentOutput,1 * RobotMap.climberMotorRInversion);
            }    
        } else {
            Robot.climberMotorRight.set(ControlMode.PercentOutput,-0);
        }

        // Check the current draw after we move the motors
        checkCurrent();
    }   

  	/************************************************************************
     ************************************************************************/

    public void LowerClimber() {

        checkCurrent();

        LowerLeftClimber(true);
        LowerRightClimber(true);
        
        checkCurrent();
    }

  	/************************************************************************
     ************************************************************************/

    public void LowerLeftClimber(boolean stopAtZero) {
        limitCountLeft--;
        // We don't have the limit switch right now.
        boolean useLimitSwitch=false;

        // Need to use encoder to track retraction.
        double posLeft = getLeftPos();
       if (Robot.leftClimbLimit.get() == true && useLimitSwitch) {
            // Stop lowering left arm
            // zero encoder
            Robot.climberMotorLeft.set(ControlMode.PercentOutput,0);
            Robot.climberMotorLeft.setSelectedSensorPosition(0);
        } else {
            if ( limitCountLeft <= 0 && ( !stopAtZero || posLeft > 0)) {
                if (posLeft<10000) {
                    // Slow down as we get close to the bottom
                    Robot.climberMotorLeft.set(ControlMode.PercentOutput, -0.5 * RobotMap.climberMotorLInversion);
                } else {    
                    Robot.climberMotorLeft.set(ControlMode.PercentOutput, -1 * RobotMap.climberMotorLInversion);
                }    
                if (posLeft < 0) {
                    Robot.climberMotorLeft.setSelectedSensorPosition(0);
                }
            } else {
                Robot.climberMotorLeft.set(ControlMode.PercentOutput,-0);
            }
        }
    }

   	/************************************************************************
     ************************************************************************/

    public void LowerRightClimber(boolean stopAtZero) {
        limitCountRight--;
        // We don't have the limit switch right now.
        boolean useLimitSwitch=false;

        // Check the current draw before we move the motors
        checkCurrent();

        double posRight = getRightPos();

        if (Robot.rightClimbLimit.get() == true && useLimitSwitch) {
            // Stop lowering right arm if the limit switch is tripped.
            // zero encoder
            Robot.climberMotorRight.set(ControlMode.PercentOutput,0);
            Robot.climberMotorRight.setSelectedSensorPosition(0);
        } else {
            if ( limitCountRight <= 0 && ( !stopAtZero || posRight > 0)) {
                if (posRight<10000) {
                    // Slow down as we get close to the bottom
                    Robot.climberMotorRight.set(ControlMode.PercentOutput, -0.5 * RobotMap.climberMotorRInversion);
                } else {    
                    Robot.climberMotorRight.set(ControlMode.PercentOutput, -1 * RobotMap.climberMotorRInversion);
                }    
                if (posRight < 0) {
                    Robot.climberMotorRight.setSelectedSensorPosition(0);
                }
            } else {
                Robot.climberMotorRight.set(ControlMode.PercentOutput,0);
            }
        }

        // Check the current draw after we move the motors
        checkCurrent();
    }



  	/************************************************************************
     ************************************************************************/

     public void ResetClimberEncoder() {
        Robot.climberMotorLeft.setSelectedSensorPosition(0);
        Robot.climberMotorRight.setSelectedSensorPosition(0); 
     }
}