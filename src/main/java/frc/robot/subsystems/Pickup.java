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
import frc.robot.Robot;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**********************************************************************************
 **********************************************************************************/

public class Pickup extends SubsystemBase {
	private DoubleSolenoid flapSolenoid;
	int inversion=1;

	/************************************************************************
	 ************************************************************************/

	public Pickup() {
		// Register this subsystem with command scheduler and set the default command
		CommandScheduler.getInstance().registerSubsystem(this);
		setDefaultCommand(new PickupControl(this));

        // Map the intake solenoid, Rev Robotics PCM on CANID RobotMap.PneumaticID
	}

	/************************************************************************
	 ************************************************************************/

	public void periodic() {}

	/************************************************************************
	 ************************************************************************/

	public void DeployPickup() { 
        Robot.PickupSolenoid.set(DoubleSolenoid.Value.kReverse);
	}


	/************************************************************************
	 ************************************************************************/

    public void RetractPickup() { 
        Robot.PickupSolenoid.set(DoubleSolenoid.Value.kForward);

	}

    /************************************************************************
	 ************************************************************************/

	 	public void pickupIntake() { 
		    Robot.pickupMotor.set(.25 * inversion);
	}

	/************************************************************************
	 ************************************************************************/

     public void pickupEject() { 
		Robot.pickupMotor.set(-.5 * inversion);
	}
    /************************************************************************
	 ************************************************************************/

	 public void cancel() {
		Robot.pickupMotor.set(0);
		Robot.pickupMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}


    /************************************************************************
	 ************************************************************************/

	 public void resetEncoders() {
		// Need to use encoders for the NEOs
		Robot.pickupRelativeEncoder.setPosition(0);
	}

    /************************************************************************
	 ************************************************************************/

	 public double getPos() {
		// Need to use encoders for the NEOs
		double pos = Robot.pickupRelativeEncoder.getPosition() * inversion;
		//SmartDashboard.putNumber("Pickup POS", pos);
		return(pos);
		
	}

}