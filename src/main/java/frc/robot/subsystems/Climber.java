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

import frc.robot.RobotMap;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**********************************************************************************
 **********************************************************************************/

public class Climber extends SubsystemBase {
	boolean pickupDebug = false;
	double pickupRPM;
	int called = 0;

	static final double extendedPosition=50000;
	static final double retractedPosition=-50000;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Pickup CAN Motor
    CANSparkMax ClimberMotor = new CANSparkMax(RobotMap.ClimberCanID, CANSparkMax.MotorType.kBrushless);
    RelativeEncoder ClimberMotorEncoder = ClimberMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, RobotMap.NeoTicksPerRotation);

    DigitalInput climberBottomLimit = new DigitalInput(6);

	/************************************************************************
	 ************************************************************************/

	public Climber() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new ClimberControl(this));
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {
	}

	/************************************************************************
	 ************************************************************************/

	private void runMotor(double speed) {
		ClimberMotor.set(speed);
	}

 	/************************************************************************
	 ************************************************************************/

	private double getPosition() {
		double pos=ClimberMotorEncoder.getPosition();

		SmartDashboard.putNumber("Climber Position",pos);

		return(pos);
	}

 	/************************************************************************
	 ************************************************************************/

	private void setPosition(double value) {
		ClimberMotorEncoder.setPosition(value);
	}

	/************************************************************************
	 ************************************************************************/

	public boolean extendClimber(double speed) {
		
		if (speed < 0) { 
			return(true);
		}

		// Check Thrower Position before moving Climber
		if (getPosition() < extendedPosition ) {
    		runMotor(speed);
			return(false);
		} else {
			cancel();
			return(true);
		}	
	}

    /************************************************************************
	 ************************************************************************/

	public boolean retractClimber(double speed) {
		boolean useLimitSwiches=false;

		if (speed > 0) { 
			return(true);
		}

		// Check Thrower Position before moving Climber
		if (getPosition() > retractedPosition ) {
			if (climberBottomLimit.get() == true && useLimitSwiches) {
       		    cancel();
				setPosition(50);
				return(true);
			} else {	
     		    runMotor(speed);
				return(false);
			}	
		} else {
     		cancel();
			return(true);
		}	
	}

	/************************************************************************
	 ************************************************************************/

	public void cancel() {
		runMotor(0);
	}
}
