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

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.*;	
import frc.robot.JoystickWrapper;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.CANSparkMax;

/**********************************************************************************
 **********************************************************************************/

public class DriverControl extends CommandBase {
	JoystickWrapper driveJoystick;
	boolean driveStraight=false;
	double straightDegrees = 0;

	/**********************************************************************************
	 **********************************************************************************/
	
    public DriverControl(WestCoastDrive subsystem) {
		addRequirements(subsystem);
		driveJoystick = new JoystickWrapper(Robot.oi.driveController, 0.15);
    }

	/**********************************************************************************
	 **********************************************************************************/
	
	@Override
	public void initialize() {
		Robot.stopAutoCommand();
	}    

	/**********************************************************************************
	 * Called every tick (20ms)
	 **********************************************************************************/
	
	@Override
	public void execute() {
		int multiplier=1;

		// X buttom aborts any running auto commands
		if (driveJoystick.isXButton()) {
			Robot.stopAutoCommand();
		}

		double pitch = Robot.navxMXP.getPitch();
		double tilt1 = Robot.navxMXP.getRawGyroX();
		double tilt2= Robot.navxMXP.getRawGyroY();

        SmartDashboard.putNumber("NavX Pitch",pitch);
        SmartDashboard.putNumber("NavX gyroX",tilt1);
		SmartDashboard.putNumber("NavX gyroY",tilt2);

		SmartDashboard.putBoolean("isAutoCommand",Robot.isAutoCommand);

		if (Robot.internalData.isAuto() || Robot.isAutoCommand) {
			// Ignore user controls during Autonomous
			return;
		}

		// Get stick inputs
        double FB = driveJoystick.getLeftStickY();
        double LR = driveJoystick.getRightStickX();

		// Slow down the driver controls when the left trigger is pressed
		if (driveJoystick.getLeftTrigger() > .5) {
			LR=LR*.7;
			FB=FB*.3;
		}

		// Apply motor braking when the right trigger is pressed
		if (driveJoystick.getRightTrigger() > .5) {
			Robot.driveBase.brakesOn();
			if (driveStraight != true) {
                // Get the current angle from the Navx 
				straightDegrees = Robot.navxMXP.getAngle();      
				driveStraight = true;
			}
		} else {
			if (driveStraight == true) {
			    driveStraight = false;
				Robot.driveBase.brakesOff();
			}	
		}	

		if (driveJoystick.isYButton()) {
			if ( Robot.doAutoCommand() ) {
				Robot.autoCommand=new AutoDriveTest();
				Robot.autoCommand.schedule();
			};
		}

		if (driveJoystick.isYButton()) {
			Robot.driveBase.resetEncoders();
		}

		if (driveJoystick.isLShoulderButton()) {
			multiplier=2;
		}

		// Shift the Robot Left
		if (driveJoystick.getPovLeft()) {
			if ( Robot.doAutoCommand() ) {
				Robot.autoCommand=new AutoMoveLeft(multiplier);
				Robot.autoCommand.schedule();
			}	
		}

		// Shift the Robot right
		if (driveJoystick.getPovRight()) {
			if ( Robot.doAutoCommand() ) {
				Robot.autoCommand=new AutoMoveRight(multiplier);
				Robot.autoCommand.schedule();
	        }			
		}

		// Auto balance the robot
		if (driveJoystick.isAButton()) {
			if ( Robot.doAutoCommand() ) {
				Robot.autoCommand=new AutoClimbBalance();
				Robot.autoCommand.schedule();
			}	
		}      

		if (driveJoystick.isAButton()) {
			if ( Robot.doAutoCommand() ) {
				Robot.autoCommand=new AutoTurn180();
				Robot.autoCommand.schedule();
			}	
		}      

		// Log the Joystick X,Y Axis to the SmartDashboard.
		//SmartDashboard.putNumber("JoyStick Y Axis",FB);
		//SmartDashboard.putNumber("JoyStick X Axis",LR);

		//SmartDashboard.putNumber("robotTurn",Robot.robotTurn);
		//SmartDashboard.putNumber("robotDrive",Robot.robotDrive);

		if (Robot.isAutoCommand) {
			// Don't do anything during auto commands
		} else if (Robot.targetType == Robot.targetTypes.TargetSeek) {
			// If we are seeking the throwing target, ignore the driver input
			Robot.driveBase.Drive(Robot.robotDrive,Robot.robotTurn);
		} else {
			// Set drivebase speed based on user input
			Robot.driveBase.Drive(FB,LR, driveStraight, straightDegrees);

		}
	}

	/**********************************************************************************
	 * Returns true if command finished
	 **********************************************************************************/

	@Override
	public boolean isFinished() {
		return false;
	}

	/**********************************************************************************
	 * Called once after isFinished returns true
	 **********************************************************************************/

	 @Override
	public void end(boolean isInterrupted) {
	}  
}
