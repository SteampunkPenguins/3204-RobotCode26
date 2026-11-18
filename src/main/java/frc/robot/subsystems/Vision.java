package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * Reads AprilTag pose estimates from the Limelight (MegaTag2) and feeds them
 * into the swerve drivetrain's pose estimator.
 *
 * MegaTag2 needs the robot's heading every loop, so this subsystem sends the
 * drivetrain's yaw to the Limelight before asking it for a pose.
 */
public class Vision extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final String name = Constants.Vision.kLimelightName;

    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        var state = drivetrain.getState();
        double yawDeg = state.Pose.getRotation().getDegrees();
        double yawRateDegPerSec = Units.radiansToDegrees(state.Speeds.omegaRadiansPerSecond);

        // 1. Tell the Limelight which way the robot is facing (required for MegaTag2).
        LimelightHelpers.SetRobotOrientation(name, yawDeg, yawRateDegPerSec, 0, 0, 0, 0);

        // 2. Ask it for a pose. Always blue-origin, regardless of alliance.
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        boolean accepted = false;
        if (estimate != null
                && estimate.tagCount > 0
                // Spinning fast makes the vision pose unreliable.
                && Math.abs(yawRateDegPerSec) < Constants.Vision.kMaxYawRateDegPerSec) {

            // MegaTag2 gets heading from the gyro, so don't let vision change the heading.
            drivetrain.addVisionMeasurement(
                    estimate.pose,
                    estimate.timestampSeconds,
                    VecBuilder.fill(Constants.Vision.kXYStdDev, Constants.Vision.kXYStdDev, 9999999));
            accepted = true;
        }

        SmartDashboard.putBoolean("Vision/Has Target", LimelightHelpers.getTV(name));
        SmartDashboard.putNumber("Vision/Tag Count", estimate == null ? 0 : estimate.tagCount);
        SmartDashboard.putBoolean("Vision/Pose Accepted", accepted);
    }
}
