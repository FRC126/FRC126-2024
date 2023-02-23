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
//import com.revrobotics.CANSparkMax;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**********************************************************************************
 **********************************************************************************/

public class Flap extends SubsystemBase {
	private DoubleSolenoid flapSolenoid;

	/************************************************************************
	 ************************************************************************/

	public Flap() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new FlapControl(this));

        // Map the intake solenoid, Rev Robotics PCM on CANID RobotMap.PneumaticID
        flapSolenoid = new DoubleSolenoid(RobotMap.PneumaticID,
		                                  PneumaticsModuleType.REVPH,3,4);	
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

	/************************************************************************
	 ************************************************************************/

	public void DeployFlap() { 
        flapSolenoid.set(DoubleSolenoid.Value.kForward);
	}


	/************************************************************************
	 ************************************************************************/

    public void RetractFlap() { 
        flapSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

    /************************************************************************
	 ************************************************************************/

	 public void cancel() {
	}

}