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


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

public class AutoPlaceConeHigh extends SequentialCommandGroup {
    public AutoPlaceConeHigh(int action) {
        /**********************************************************************************
         **********************************************************************************/

        addCommands(
            // Backup to allow for room to raise the arm.
            new DriveDistance(-6, 75),

            // Drive Backwards, Raise and Extend the Arm.
            new ParallelCommandGroup(
                new DriveDistance(-12, 250),
                new MoveTowerArm(RobotMap.towerArmExtendedHighPos-5, 250),
                new MoveArmExtension(RobotMap.armExtendedPlacePos, 250)
            ),

            // Drive Forwards
            new DriveDistance(17, 250)
        );

        if (action != 0) {
            // If we are in Autonomous mode, then need to lower and retract the arm            
            addCommands(
                // Lower the arm a little    
                new MoveTowerArm(RobotMap.towerArmExtendedHighPos-20, 150),

                // Open the grabber to drop the cone
                new MoveGrabber(RobotMap.grabberConePos+100, 100),

                // Drive Backwards, retact the arm, and start closing the grabber
                new ParallelCommandGroup(
                    new DriveDistance(-12, 150),
                    new MoveArmExtension(RobotMap.armRetractedPos+40, 120),
                    new MoveGrabber(RobotMap.grabberClosedPos+25, 120)
                )    
           );
        } else {
            // This is a command during teleop, lower the arm a little and give control back
            addCommands(
                new MoveTowerArm(RobotMap.towerArmExtendedHighPos-15, 100)
            );    
        }

        if (action == 5) {
            // Action 5 is do nothing, just retract the arm.
            addCommands(             
                // Backup a little to give room to lower the arm
                new DriveDistance(-6, 75),
                // Fully retract the arm
                new ParallelCommandGroup(
                    new MoveArmExtension(RobotMap.armRetractedPos, 200),
                    new MoveGrabber(RobotMap.grabberClosedPos+5, 200),
                    new MoveTowerArm(RobotMap.towerArmRetractedPos, 200)
                )
            );
        } else if (action == 1) {
            // Action 1 is auto balance
            addCommands(             
                // Backup a little to give room to lower the arm
                // new DriveDistance(-6, 75),
                // Fully retract the arm while auto balancing
                new ParallelCommandGroup(
                    new MoveArmExtension(RobotMap.armRetractedPos, 250),
                    new MoveGrabber(RobotMap.grabberClosedPos+5, 250),
                    new MoveTowerArm(RobotMap.towerArmRetractedPos, 250),
                    new ClimbAndBalanceBackwards(5000)
                )
            );
        } else if (action == 2) {
            // Action 2 is we are are on the inside position, and want to back up and grab a cube
            addCommands(
                // Backup a little to give room to lower the arm
                new DriveDistance(-6, 75),
                // Fully retract the arm while backing out of the community zone
                new ParallelCommandGroup(
                    new MoveArmExtension(RobotMap.armRetractedPos, 100),
                    new MoveGrabber(RobotMap.grabberClosedPos+5, 100),
                    new MoveTowerArm(RobotMap.towerArmRetractedPos, 100),
                    new DriveDistance(-12*8,300)
                ),
                // Finish backing up and lower the pickup and run it.
                new TurnDegreesBetter(175,250),

                new ParallelCommandGroup(
                    new DriveDistance(-12*3.5,150),
                    new RunPickup(1,200)
                )    
            );
        } else if (action == 4 || action == 3 ) {
            int turnDirection=1;
            if (action == 3) { turnDirection = -1; }
            SmartDashboard.putNumber("AutoDebug ACtion",action);

            // Action 3 is we are on the outside position, blue alliance
            addCommands(
                new DriveDistance(-6, 75),
                // Fully retract the arm while backing turning to avoid obsticle
                new ParallelCommandGroup(
                    new MoveArmExtension(RobotMap.armRetractedPos, 50),
                    new MoveGrabber(RobotMap.grabberClosedPos+5, 50),
                    new MoveTowerArm(RobotMap.towerArmRetractedPos, 50),
                    new TurnDegreesBetter(15 * turnDirection,100)
                ),
                // Fully retract the arm while backing up around obsticle
                new ParallelCommandGroup(
                    new MoveArmExtension(RobotMap.armRetractedPos, 50),
                    new MoveGrabber(RobotMap.grabberClosedPos+5, 50),
                    new MoveTowerArm(RobotMap.towerArmRetractedPos, 50),
                    new DriveDistance(-10,76)
                ),
                // turn back to a straight line
                //new TurnDegreesBetter(-15 * turnDirection,100),

                new ParallelCommandGroup(
                    new MoveArmExtension(RobotMap.armRetractedPos, 50),
                    new MoveGrabber(RobotMap.grabberClosedPos+5, 50),
                    new MoveTowerArm(RobotMap.towerArmRetractedPos, 50),
                    new DriveDistance(-12*7,500)
                ),                // Finish backing up and lower the pickup and run it.
                new TurnDegreesBetter(10 * turnDirection,75),
                new ParallelCommandGroup(
                    new DriveDistance(-12*3,300),
                    new RunPickup(1,300)
                )
            );
        }
 
        addCommands(
            new FinishAuto()        
        );
    }       
}
