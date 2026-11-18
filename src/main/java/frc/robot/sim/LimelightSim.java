package frc.robot.sim;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * A fake Limelight for the simulator. SIMULATION ONLY.
 *
 * Each loop it works out which 2026 field AprilTags the camera could see from
 * the robot's simulated position, then writes a MegaTag2 pose to the same
 * NetworkTables entries a real Limelight uses. Vision.java reads them through
 * LimelightHelpers exactly as it would on the real robot.
 */
public class LimelightSim {

    /** Max angle between a tag's face and the camera before the tag can't be read. */
    private static final double kMaxViewAngleDeg = 70;

    /** Fake position error per meter of distance to the tags (meters). */
    private static final double kNoisePerMeter = 0.02;

    private final NetworkTable table =
            NetworkTableInstance.getDefault().getTable(Constants.Vision.kLimelightName);

    private final AprilTagFieldLayout field =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private final Transform3d robotToCamera = new Transform3d(
            new Translation3d(
                    Constants.Vision.kCamForwardMeters,
                    Constants.Vision.kCamSideMeters,
                    Constants.Vision.kCamUpMeters),
            // WPILib pitch is + nose DOWN, Limelight pitch is + UP, so flip it.
            new Rotation3d(
                    Math.toRadians(Constants.Vision.kCamRollDeg),
                    Math.toRadians(-Constants.Vision.kCamPitchDeg),
                    Math.toRadians(Constants.Vision.kCamYawDeg)));

    private final Random random = new Random();
    private final Field2d simField = new Field2d();

    public LimelightSim() {
        SmartDashboard.putData("Vision/Sim Field", simField);
    }

    /** Call every loop from Robot.simulationPeriodic() with the sim robot pose. */
    public void update(Pose2d robotPose) {
        Pose3d cameraPose = new Pose3d(robotPose).transformBy(robotToCamera);

        List<double[]> seen = new ArrayList<>(); // {id, tx, ty, distance}
        List<Pose2d> seenTagPoses = new ArrayList<>();

        for (AprilTag tag : field.getTags()) {
            // Where the tag is relative to the camera (x forward, y left, z up).
            Translation3d inCamera = tag.pose.relativeTo(cameraPose).getTranslation();
            double distance = inCamera.getNorm();
            if (inCamera.getX() <= 0 || distance > Constants.Vision.kSimMaxTagDistanceMeters) {
                continue;
            }

            // Inside the camera's field of view?
            double tx = Math.toDegrees(Math.atan2(inCamera.getY(), inCamera.getX()));
            double ty = Math.toDegrees(Math.atan2(inCamera.getZ(), inCamera.getX()));
            if (Math.abs(tx) > Constants.Vision.kSimHorizontalFovDeg / 2
                    || Math.abs(ty) > Constants.Vision.kSimVerticalFovDeg / 2) {
                continue;
            }

            // Is the tag's printed side facing the camera? (A tag's +X axis points out of its face.)
            Translation3d tagFacing = new Translation3d(1, 0, 0).rotateBy(tag.pose.getRotation());
            Translation3d tagToCamera = cameraPose.getTranslation().minus(tag.pose.getTranslation());
            double cosAngle = dot(tagFacing, tagToCamera) / tagToCamera.getNorm();
            if (cosAngle < Math.cos(Math.toRadians(kMaxViewAngleDeg))) {
                continue;
            }

            seen.add(new double[] {tag.ID, tx, ty, distance});
            seenTagPoses.add(tag.pose.toPose2d());
        }

        simField.setRobotPose(robotPose);
        simField.getObject("Visible Tags").setPoses(seenTagPoses);

        if (seen.isEmpty()) {
            table.getEntry("tv").setDouble(0);
            table.getEntry("botpose_orb_wpiblue").setDoubleArray(new double[0]);
            table.getEntry("botpose_wpiblue").setDoubleArray(new double[0]);
            return;
        }

        double avgDist = seen.stream().mapToDouble(s -> s[3]).average().orElse(0);
        double noise = kNoisePerMeter * avgDist;
        double x = robotPose.getX() + random.nextGaussian() * noise;
        double y = robotPose.getY() + random.nextGaussian() * noise;

        // Same layout a real Limelight publishes:
        // [x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan, avgDist, avgArea,
        //  then 7 values per tag: id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
        double[] data = new double[11 + 7 * seen.size()];
        data[0] = x;
        data[1] = y;
        data[5] = robotPose.getRotation().getDegrees();
        data[6] = 0; // latency: the pose above is "now", so report none
        data[7] = seen.size();
        data[9] = avgDist;
        for (int i = 0; i < seen.size(); i++) {
            double[] s = seen.get(i);
            int b = 11 + 7 * i;
            data[b] = s[0];
            data[b + 1] = s[1];
            data[b + 2] = s[2];
            data[b + 4] = s[3];
            data[b + 5] = s[3];
        }

        table.getEntry("tv").setDouble(1);
        table.getEntry("tid").setDouble(seen.get(0)[0]);
        table.getEntry("tx").setDouble(seen.get(0)[1]);
        table.getEntry("ty").setDouble(seen.get(0)[2]);
        table.getEntry("botpose_orb_wpiblue").setDoubleArray(data); // MegaTag2
        table.getEntry("botpose_wpiblue").setDoubleArray(data);     // MegaTag1 (same fake data)

        simField.getObject("Vision Pose").setPose(new Pose2d(x, y, robotPose.getRotation()));
    }

    private static double dot(Translation3d a, Translation3d b) {
        return a.getX() * b.getX() + a.getY() * b.getY() + a.getZ() * b.getZ();
    }
}
