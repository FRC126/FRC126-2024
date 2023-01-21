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
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**********************************************************************************
 **********************************************************************************/

public class BallIntake extends SubsystemBase {
    int delay=0;
	private DoubleSolenoid intakeSolenoid;
	
	/************************************************************************
	 ************************************************************************/

     public BallIntake() {
        // Register this subsystem with command scheduler and set the default command
        CommandScheduler.getInstance().registerSubsystem(this);
        setDefaultCommand(new IntakeControl(this));

        // Map the intake solenoid, Rev Robotics PCM on CANID 2
        intakeSolenoid = new DoubleSolenoid(2,PneumaticsModuleType.REVPH,11,12);
    }

	/************************************************************************
	 ************************************************************************/
    
    public void periodic() {}

	/************************************************************************
     * Run Intake Wheels
	 ************************************************************************/

    public void IntakeSpeed(double speed) {
        Robot.intakeMotor1.set(speed);
        if (!Robot.throwerRunning) {
            Robot.intakeMotor2.set(speed*-1);
        }    
    }   

  	/************************************************************************
	 ************************************************************************/

     public void IntakeRun() {
        IntakeSpeed(-0.7);
    }

  	/************************************************************************
	 ************************************************************************/

     public void IntakeReverse() {
        IntakeSpeed(0.7);
    }

  	/************************************************************************
	 ************************************************************************/

     public void IntakeStop() {
        IntakeSpeed(0);
    }

  	/************************************************************************
	 ************************************************************************/

    public void neutralIntake() {
        intakeSolenoid.set(DoubleSolenoid.Value.kOff);
    }

  	/************************************************************************
	 ************************************************************************/
    
     public void ExtendIntake() {      
        intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

  	/************************************************************************
	 ************************************************************************/

    public void RetractIntake() {      
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }
}

