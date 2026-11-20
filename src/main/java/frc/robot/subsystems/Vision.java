package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
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

    /* Latest results, shown on the dashboard */
    private boolean hasTarget = false;
    private int tagCount = 0;
    private boolean poseAccepted = false;

    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Tell the Limelight where it is mounted on the robot.
        LimelightHelpers.setCameraPose_RobotSpace(name,
                Constants.Vision.kCamForwardMeters,
                Constants.Vision.kCamSideMeters,
                Constants.Vision.kCamUpMeters,
                Constants.Vision.kCamRollDeg,
                Constants.Vision.kCamPitchDeg,
                Constants.Vision.kCamYawDeg);
    }

    /**
     * Robot position from MegaTag1, which works out position AND heading from the
     * tags alone (MegaTag2 needs a correct heading first). Use this to find out
     * where the robot is at the start of auto. Empty if fewer than minTags tags are seen.
     */
    public Optional<Pose2d> getMegaTag1Pose(int minTags) {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (estimate == null || estimate.tagCount < minTags) {
            return Optional.empty();
        }
        return Optional.of(estimate.pose);
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

        // Saved for the dashboard (Dashboard.java publishes these)
        hasTarget = LimelightHelpers.getTV(name);
        tagCount = estimate == null ? 0 : estimate.tagCount;
        poseAccepted = accepted;
    }

    //---------- Dashboard Accessors (read by Dashboard.java) ----------//

    /** True when the Limelight sees at least one AprilTag. */
    public boolean hasTarget() {
        return hasTarget;
    }

    /** How many AprilTags were used in the latest pose estimate. */
    public int getTagCount() {
        return tagCount;
    }

    /** True when the latest vision pose was used to correct the robot's position. */
    public boolean isPoseAccepted() {
        return poseAccepted;
    }
}
