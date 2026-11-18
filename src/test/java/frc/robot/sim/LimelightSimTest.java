package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

class LimelightSimTest {

    private static final String NAME = Constants.Vision.kLimelightName;

    /** Robot placed `distance` meters in front of a tag, either facing it or facing away. */
    private static Pose2d poseInFrontOfTag(int tagId, double distance, boolean facingTag) {
        var field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        Pose2d tag = field.getTagPose(tagId).orElseThrow().toPose2d();
        Translation2d spot = tag.getTranslation()
                .plus(new Translation2d(distance, tag.getRotation()));
        Rotation2d heading = facingTag
                ? tag.getRotation().plus(Rotation2d.k180deg)
                : tag.getRotation();
        return new Pose2d(spot, heading);
    }

    @Test
    void seesTagWhenFacingIt() {
        var sim = new LimelightSim();
        Pose2d robot = poseInFrontOfTag(10, 2.0, true);
        sim.update(robot);

        assertTrue(LimelightHelpers.getTV(NAME));
        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(NAME);
        assertTrue(estimate.tagCount >= 1);
        // Noise is small (2 cm per meter), so the estimate should be close to the true pose.
        assertEquals(robot.getX(), estimate.pose.getX(), 0.3);
        assertEquals(robot.getY(), estimate.pose.getY(), 0.3);
    }

    @Test
    void doesNotSeeTagBehindIt() {
        var sim = new LimelightSim();
        // Facing away from hub tag 10 means looking at the red wall, where tags 15/16 are visible.
        sim.update(poseInFrontOfTag(10, 2.0, false));

        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(NAME);
        for (var tag : estimate.rawFiducials) {
            assertFalse(tag.id == 10, "tag 10 is behind the camera and should not be seen");
        }
    }

    @Test
    void seesNothingWhenNoTagInView() {
        var sim = new LimelightSim();
        // Middle of the field, looking along the centerline toward the side wall: no tags there.
        sim.update(new Pose2d(8.27, 4.0, Rotation2d.kCCW_90deg));

        assertFalse(LimelightHelpers.getTV(NAME));
        assertEquals(0, LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(NAME).tagCount);
    }
}
