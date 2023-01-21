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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.JoystickWrapper;

/**********************************************************************************
 **********************************************************************************/

public class PixyCameraData extends CommandBase {
	static int count=0;
    int loop_count=0;
	int missed_count=0;
	int directionX;
	int directionY;

	public PixyCameraData(PixyVision subsystem) {
		// Use requires() here to declare subsystem dependencies
		addRequirements(subsystem);
		directionX = 1;
		directionY = 1;
	}

	// Run before command starts 1st iteration
	@Override
	public void initialize() {
	}

	// Called every tick (20ms)
	@SuppressWarnings("static-access")
	@Override
	public void execute() {
        //Turn on the LED's on the PixyCam 
		//Robot.vision.setLamp(True,True);
		int objectId=Robot.objectId;

		JoystickWrapper driveJoystick = new JoystickWrapper(Robot.oi.driveController, 0.05);

		if (driveJoystick.getPovUp()) {
			Robot.targetType = Robot.targetTypes.BallSeek;
			Robot.objectId=1;
		} else if (driveJoystick.getPovDown() ) {
			Robot.targetType = Robot.targetTypes.BallSeek;
			Robot.objectId=2;
		} else if (driveJoystick.getPovLeft() ) {
			Robot.targetType = Robot.targetTypes.PixyTargetSeek;
			Robot.objectId=3;
		} else {
			Robot.targetType = Robot.targetTypes.NoTarget;
		}
		
		if (Robot.targetType == Robot.targetTypes.NoTarget) {
            Robot.robotDrive=0;
			Robot.robotTurn=0;
		}
		
		count++;
        SmartDashboard.putNumber("Pixy Count: ", count);

		//if (Robot.trackTarget != Robot.targetTypes.ballTarget) {
		//	Robot.pixyVision.trackTargetPosition(objectId);
		//	Robot.pixyVision.setLamp(false,false);
		//	return;
		//}

		// Track Specified object ID
		// 1 - Power Cell
		// 2 - Throwing Target
		// 
		// TODO hook this up to a button or something so that we
		// track the power cell, or the throwing target

		// Get the data for requested object from the camera
		Robot.pixyVision.getItems(objectId,1);

		// Report the object data to the smart dashboard.
		SmartDashboard.putNumber("Vision X: ", Robot.pixyVision.packetData[objectId].X);
		//SmartDashboard.putNumber("Vision Y: ", Robot.vision.packetData[objectId].Y);
		SmartDashboard.putNumber("Vision H: ", Robot.pixyVision.packetData[objectId].Height);
		//SmartDashboard.putNumber("Vision W: ", Robot.vision.packetData[objectId].Width);
		SmartDashboard.putBoolean("Vision V: ", Robot.pixyVision.packetData[objectId].isValid);
		SmartDashboard.putNumber("Servo X: ", Robot.pixyVision.getServoX());
		SmartDashboard.putNumber("Servo Y: ", Robot.pixyVision.getServoY());
        SmartDashboard.putNumber("TargetID: ", Robot.objectId);
		if ( Robot.targetType == Robot.targetTypes.BallSeek) {
            SmartDashboard.putString("TargetType: ", "BallSeek");
		}
		if ( Robot.targetType == Robot.targetTypes.BallSeek) {
            SmartDashboard.putString("TargetType: ", "BallSeek");
		}
		if ( Robot.targetType == Robot.targetTypes.NoTarget) {
            SmartDashboard.putString("TargetType: ", "NoTarget");
		}
        if ( Robot.targetType == Robot.targetTypes.PixyTargetSeek) {
            SmartDashboard.putString("TargetType: ", "PixyTargetSeek");
		}

        if (Robot.targetType == Robot.targetTypes.BallSeek ||
		    Robot.targetType == Robot.targetTypes.PixyTargetSeek ) {
		    Robot.pixyVision.setLamp(true,false);
		} else {
			Robot.pixyVision.setLamp(false,false);
			return;
		}
		
	    if (Robot.pixyVision.packetData[objectId].isValid) {
			// If the object is valid then turn on the LED and use the servos
			// to center the object in the camera view
			Robot.pixyVision.setLED(0,255,0); 

			if (Robot.pixyVision.packetData[objectId].Y < 80) {
				// if the object is below the center of the camera, move the
				// camera down
				Robot.pixyVision.incrServoY(-5);
			}
			if (Robot.pixyVision.packetData[objectId].Y > 120) {
				// if the object is above the center of the camera, move the
				// camera up
				Robot.pixyVision.incrServoY(5);
			}
			if (Robot.pixyVision.packetData[objectId].X < 145) {
				// if the object is to the left of the center of the camera, move the
				// camera left
				Robot.pixyVision.incrServoX(5);
			}
			if (Robot.pixyVision.packetData[objectId].X > 170) {
				// if the object is to the right of the center of the camera, move the
				// camera right
				Robot.pixyVision.incrServoX(-5);
			}

			loop_count=0;



			Robot.pixyVision.trackTargetPosition(objectId);
		} else {
			// Set the LED to signify object was not found.
			Robot.pixyVision.setLED(255,0,0); 

			loop_count++;
			if (loop_count == 5) {
				// After 250 iterations of not seeing an object, recenter the camera.
				Robot.pixyVision.centerServo();
			}
			if (loop_count > 10) {
			    // Scan for a target
			    Robot.pixyVision.incrServoX(8 * directionX);
			    Robot.pixyVision.incrServoY(4 * directionY);

			    if ( Robot.pixyVision.getServoX() > 450) {
				   Robot.pixyVision.setServoX(450);
				   directionX = -1;
			    }
			    if ( Robot.pixyVision.getServoX() < 50) {
			    	Robot.pixyVision.setServoX(50);
			    	directionX = 1;
			    }	
			    if ( Robot.pixyVision.getServoY() > 490) {
				    Robot.pixyVision.setServoY(490);
				    directionY = -1;
	        	}
			    if ( Robot.pixyVision.getServoY() < 300) {
				    Robot.pixyVision.setServoY(300);
				    directionY = 1;
				}
				//System.out.println("dY: " + directionY + " dX: " + directionX + " lC: " + loop_count + " sX: " + Robot.vision.getServoX() + " sY: " + Robot.vision.getServoY() );
			}	
		}

		// Set the calculated servo position 
		Robot.pixyVision.setServo();	
	}

	// Returns true if command finished
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
	}
}
