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
import frc.robot.Drivetrain;
import frc.robot.JoystickWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PrototypeControl extends Command {
	JoystickWrapper driveJoystick;
	boolean driveStraight=false;
	double straightDegrees = 0;

	/**********************************************************************************
	 **********************************************************************************/
	
    public PrototypeControl(Prototype subsystem) {
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
        double y1 = driveJoystick.getLeftStickY();
        double x1 = driveJoystick.getLeftStickX();
        double x2 = driveJoystick.getRightStickX();

		Robot.swerveDrive.Drive(y1, x1, x2);
	}

}

