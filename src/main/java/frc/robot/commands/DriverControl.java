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

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.*;	
import frc.robot.JoystickWrapper;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

public class DriverControl extends CommandBase {
	static int delay=0;
	static boolean turnAround=false;
	static double startAngle;
	static double shiftDuration = 0;
	JoystickWrapper driveJoystick;
	
	/**********************************************************************************
	 **********************************************************************************/
	
    public DriverControl(WestCoastDrive subsystem) {
		addRequirements(subsystem);
		driveJoystick = new JoystickWrapper(Robot.oi.driveController, 0.1);
    }

	/**********************************************************************************
	 **********************************************************************************/
	
	@Override
	public void initialize() {
	}    

	/**********************************************************************************
	 * Called every tick (20ms)
	 **********************************************************************************/
	
	@SuppressWarnings("static-access")
	@Override
	public void execute() {
		if (Robot.internalData.isAuto()) {
			// Ignore user controls during Autonomous
			return;
		}

		// Get stick inputs
        double FB = driveJoystick.getLeftStickY();
        double LR = driveJoystick.getRightStickX() * -1 ;

		if (driveJoystick.getLeftTrigger() > .1) {
			LR=LR*.7;
			FB=FB*.3;
		}

		if (driveJoystick.isLShoulderButton()) {
			// Shift Down Drive Train
		    if (delay <= 0 ) {
				Robot.driveBase.shiftDown();
				delay=0;
				shiftDuration = 10;
			}
			SmartDashboard.putBoolean("Shift Down",true);
		} else {
			SmartDashboard.putBoolean("Shift Down",false);
		}

		if (driveJoystick.isRShoulderButton()) {
			// Shift Up Drive Train
		    if (delay <= 0 ) {
				Robot.driveBase.shiftUp();
				delay=0;
				shiftDuration = 10;
			}	
			SmartDashboard.putBoolean("Shift Up",true);
		} else {
			SmartDashboard.putBoolean("Shift Up",false);
		}

		//Disable power drop during shifting for now
		//if(shiftDuration > 0) {
		//	Robot.driveBase.limitSpeedForShift();
		//	shiftDuration--;
		//} else {
		//	Robot.driveBase.delimitSpeed();
		//	shiftDuration = 0;
		//}

		if (driveJoystick.isXButton()) {
			// Turn 180 degrees after X Button is pressed
			if (turnAround == false) {
				startAngle = Robot.internalData.getGyroAngle();
			    turnAround=true;
			}
		}

		if (turnAround == true) {
			// If we are supposed to turn around, check the current gyro
			// angle and see if we have reached our desired position,
			// if not, keep turning
			double currAngle = Robot.internalData.getGyroAngle();
			if(currAngle < startAngle + 155) {
				LR=-0.3;
			} else {
				LR=0;
				turnAround=false;
			}
		}

		if (driveJoystick.isRStickPressButton()) {
			LR = LR *.5;
		}
		if (driveJoystick.isLStickPressButton()) {
			FB = FB *.5;
		}

		delay--;

		// Log the Joystick X,Y Axis to the SmartDashboard.
		//SmartDashboard.putNumber("JoyStick Y Axis",FB);
		//SmartDashboard.putNumber("JoyStick X Axis",LR);
		SmartDashboard.putNumber("robotTurn",Robot.robotTurn);
		SmartDashboard.putNumber("robotDrive",Robot.robotDrive);

		if (Robot.targetType == Robot.targetTypes.TargetSeek) {
			// If we are seeking the throwing target, ignore the driver input
			Robot.driveBase.Drive(Robot.robotDrive,Robot.robotTurn);
		} else {
			// Set drivebase speed based on user input
			Robot.driveBase.Drive(FB,LR);
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
