package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * This sets the yaw such that it is aligned to the driver
 * and also the alliance (blue or red)
 */
public class ResetYawCommand extends Command {
    private double targetDegrees;
    private double tolerance = 5;
    private boolean finished = false;

    public ResetYawCommand() {
        super();
        addRequirements(Robot.limeLight, Robot.swerveDrive);

        Optional<Alliance> ally = DriverStation.getAlliance();

        if (ally.isPresent()) {
            if (ally.get() == Alliance.Blue) {
                // LimeLight docs say pose is always relative to Blue alliance
                this.targetDegrees = 0;
            } else {
                this.targetDegrees = 360;
            }
        } else {
            // can't automatically set yaw
            finished = true;
        }
    }

    @Override
    public void execute() {
        Robot.limeLight.getCameraData();
        Pose2d botPose2d = Robot.limeLight.getBotPose2d();
        double degrees = botPose2d.getRotation().getDegrees();
        if (degrees < this.targetDegrees + tolerance) {
            // turn CW, TODO: is this how to turn?
            Robot.swerveDrive.Drive(0, 0, 5);
        } else if (degrees > this.targetDegrees - tolerance) {
            // turn CCW
            Robot.swerveDrive.Drive(0, 0, -5);
        } else {
            // we got to where we want to be
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (finished) {
            Robot.swerveDrive.resetYaw();
            return true;
        }
        return false;
    }
}
