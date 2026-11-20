package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Where our hub is and how the robot is placed relative to it.
 *
 * The robot's position comes from the drivetrain's pose estimator, which
 * Vision.java keeps correcting with Limelight AprilTag measurements. So the
 * distance and angle here are AprilTag-based whenever the camera can see tags,
 * and fall back to wheel odometry when it can't.
 *
 * All positions are blue-origin field coordinates (meters), the same as
 * drivetrain.getState().Pose:
 *   x = 0 at the blue alliance wall, increasing toward the red wall
 *   y = 0 at the bottom side rail
 */
public final class HubTargeting {

    private HubTargeting() {}

    /* Official 2026 field tag positions (same layout LimelightSim uses) */
    private static final AprilTagFieldLayout kField =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    //---------- Hub centers, worked out from the hub's AprilTags ----------//
    //The two tags on each hub face are NOT centered on that face, so averaging all 8 tags is off by ~9 cm.
    //Instead: X comes from the two faces that point along the field (toward the walls),
    //         Y comes from the two faces that point toward the side rails.
    public static final Translation2d kBlueHubCenter = new Translation2d(
            averageX(25, 26, 19, 20),   // faces toward the blue wall (25, 26) and toward the center (19, 20)
            averageY(21, 24, 18, 27));  // faces toward the top rail (21, 24) and bottom rail (18, 27)
    public static final Translation2d kRedHubCenter = new Translation2d(
            averageX(3, 4, 9, 10),      // faces toward the center (3, 4) and toward the red wall (9, 10)
            averageY(2, 11, 5, 8));     // faces toward the top rail (2, 11) and bottom rail (5, 8)

    //---------- Scoring zones: between our alliance wall and our trench ----------//
    //Tag 23 is on the blue trench, on the side facing the blue wall.
    //Tag 12 is on the red trench, on the side facing the red wall.
    public static final double kBlueZoneMaxX = tagX(23);
    public static final double kRedZoneMinX = tagX(12);

    /** Our alliance from the Driver Station. Blue if it isn't known yet. */
    public static Alliance alliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    /** Center of OUR hub: the red hub on red, the blue hub on blue. */
    public static Translation2d hubCenter() {
        return alliance() == Alliance.Red ? kRedHubCenter : kBlueHubCenter;
    }

    /** Straight-line distance from the robot's center to our hub's center, in meters. */
    public static double distanceToHubMeters(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(hubCenter());
    }

    /** Same as distanceToHubMeters, in feet. */
    public static double distanceToHubFeet(Pose2d robotPose) {
        return Units.metersToFeet(distanceToHubMeters(robotPose));
    }

    /**
     * The field heading the ROBOT should have so the SHOOTER points at our hub.
     * (Takes Constants.Aim.kShooterFacingDeg into account.)
     */
    public static Rotation2d headingToFaceHub(Pose2d robotPose) {
        Rotation2d towardHub = hubCenter().minus(robotPose.getTranslation()).getAngle();
        return towardHub.minus(Rotation2d.fromDegrees(Constants.Aim.kShooterFacingDeg));
    }

    /** How far the shooter is turned away from the hub center, in degrees (-180 to 180). */
    public static double aimErrorDegrees(Pose2d robotPose) {
        return robotPose.getRotation().minus(headingToFaceHub(robotPose)).getDegrees();
    }

    /** True when the shooter points at the hub center within Constants.Aim.kLinedUpToleranceDeg. */
    public static boolean isLinedUp(Pose2d robotPose) {
        return Math.abs(aimErrorDegrees(robotPose)) <= Constants.Aim.kLinedUpToleranceDeg;
    }

    /** True when the robot is between our alliance wall and our trench, where the hub can be scored. */
    public static boolean isInScoringZone(Pose2d robotPose) {
        double x = robotPose.getX();
        return alliance() == Alliance.Red ? x >= kRedZoneMinX : x <= kBlueZoneMaxX;
    }

    private static double averageX(int... ids) {
        double sum = 0;
        for (int id : ids) {
            sum += tagPose(id).getX();
        }
        return sum / ids.length;
    }

    private static double averageY(int... ids) {
        double sum = 0;
        for (int id : ids) {
            sum += tagPose(id).getY();
        }
        return sum / ids.length;
    }

    private static double tagX(int id) {
        return tagPose(id).getX();
    }

    private static edu.wpi.first.math.geometry.Pose3d tagPose(int id) {
        return kField.getTagPose(id).orElseThrow(
                () -> new IllegalStateException("AprilTag " + id + " is not in the 2026 field layout"));
    }
}
