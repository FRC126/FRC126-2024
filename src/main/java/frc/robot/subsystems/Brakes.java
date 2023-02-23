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

package frc.robot.subsystems;

//import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;


/**********************************************************************************
 **********************************************************************************/

public class Brakes extends SubsystemBase {
	private DoubleSolenoid brakeSolenoid;

	/************************************************************************
	 ************************************************************************/

	public Brakes() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new BrakeControl(this));
       
        // Map the intake solenoid, Rev Robotics PCM on CANID RobotMap.PneumaticID
        brakeSolenoid = new DoubleSolenoid(RobotMap.PneumaticID,
                                           PneumaticsModuleType.REVPH,1,2);	
    }

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

	/************************************************************************
	 ************************************************************************/

	public void ApplyBrakes() { 
        brakeSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	/************************************************************************
	 ************************************************************************/

     public void ReleaseBrakes() { 
        brakeSolenoid.set(DoubleSolenoid.Value.kReverse);
	}
    /************************************************************************
	 ************************************************************************/

	 public void cancel() {
	}
}