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

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.*;
import frc.robot.JoystickWrapper;
import edu.wpi.first.wpilibj2.command.Command;

public class PickupControl extends Command {
	JoystickWrapper operatorJoystick;
	Pickup pickup;
	static int keepRunning=0;
	static int seenCount=0;

	/**********************************************************************************
	 **********************************************************************************/

	public PickupControl(Pickup subsystem) {
		addRequirements(subsystem);
		operatorJoystick = new JoystickWrapper(Robot.oi.operatorController, 0.15);
		this.pickup = subsystem;
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

		if (Robot.internalData.isAuto() || Robot.isAutoCommand) {
			// Ignore user controls during Autonomous
			Robot.pickup.setUserRunPickup(false);
			return;
		}		

       	if (Robot.pickup.getPhotoSensor()) {
			Robot.Leds.setMode(LEDSubsystem.LEDModes.RunPickup);
		}

		if (operatorJoystick.leftTriggerPressed() || operatorJoystick.isYButton()) {
			Robot.pickup.setUserRunPickup(true);
			Robot.Leds.setMode(LEDSubsystem.LEDModes.RunPickup);
			keepRunning=50;
           	if (Robot.pickup.getPhotoSensor()) { seenCount++; }
			if (seenCount > 10) { 
				keepRunning=0;
    			this.pickup.cancel();
			} else {
    			this.pickup.pickupMotorOn();
			}
		} else if ( keepRunning > 0 ) {
			Robot.pickup.setUserRunPickup(true);
			Robot.Leds.setMode(LEDSubsystem.LEDModes.RunPickup);
			this.pickup.pickupMotorOn();	
			keepRunning--;
           	if (Robot.pickup.getPhotoSensor()) { seenCount++; }
			if (seenCount > 10) { 
				keepRunning=0;
    			this.pickup.cancel();
			} else {
    			this.pickup.pickupMotorOn();			
			}
		} else if (operatorJoystick.isLShoulderButton()) {
			Robot.pickup.setUserRunPickup(true);
			this.pickup.pickupMotorReverse();
			seenCount=0;
		} else {
			Robot.pickup.setUserRunPickup(false);
			if (!Robot.pickup.getAutoRunPickup()) {
  				this.pickup.cancel();
			}	
			seenCount=0;
		}
	}
}
