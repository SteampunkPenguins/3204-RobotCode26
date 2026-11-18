package frc.robot.commands;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * Auto: use AprilTags to find where we are, drive to the center of the field,
 * then turn around 180 degrees.
 *
 * 1. Wait (up to 1 s) for the Limelight to see a tag, then set the robot's
 *    position from vision (MegaTag1). If no tag is seen, keep the current odometry.
 * 2. PathPlanner pathfinding drives to the field center, routing around the
 *    obstacles in navgrid.json, facing away from our alliance wall. Vision keeps
 *    correcting the position the whole way (Vision.java).
 * 3. Turn in place 180 degrees, so we face back toward our alliance wall.
 */
public class VisionCenterAuto extends SequentialCommandGroup {

    /** Center of the 2026 field (16.54 m x 8.07 m), blue-origin coordinates. */
    private static final Pose2d kFieldCenter = new Pose2d(16.54 / 2, 8.07 / 2, Rotation2d.kZero);

    /** Slower than the robot's max so it's safe while testing. */
    private static final PathConstraints kConstraints = new PathConstraints(
            2.0, 2.0,                                         // m/s, m/s^2
            Units.degreesToRadians(360), Units.degreesToRadians(540)); // rad/s, rad/s^2

    private static final double kTurnToleranceDeg = 3;

    public VisionCenterAuto(CommandSwerveDrivetrain drivetrain, Vision vision) {
        SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle()
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance) // angles are field angles
                .withHeadingPID(6, 0, 0);
        Rotation2d[] turnTarget = new Rotation2d[1];

        addCommands(
            // 1. Find where we are from the AprilTags
            Commands.waitUntil(() -> vision.getMegaTag1Pose(1).isPresent()).withTimeout(1.0),
            Commands.runOnce(() -> vision.getMegaTag1Pose(1).ifPresentOrElse(
                    drivetrain::resetPose,
                    () -> DriverStation.reportWarning("VisionCenterAuto: no AprilTag seen, using odometry", false))),

            // 2. Drive to the field center, facing away from our alliance wall
            Commands.defer(
                    () -> AutoBuilder.pathfindToPose(
                            new Pose2d(kFieldCenter.getTranslation(), awayFromOurWall()),
                            kConstraints, 0.0),
                    Set.of(drivetrain)),

            // 3. Turn around 180 degrees in place
            Commands.runOnce(() -> turnTarget[0] =
                    drivetrain.getState().Pose.getRotation().plus(Rotation2d.k180deg)),
            drivetrain.applyRequest(() -> faceAngle.withVelocityX(0).withVelocityY(0)
                            .withTargetDirection(turnTarget[0]))
                    .until(() -> Math.abs(drivetrain.getState().Pose.getRotation()
                            .minus(turnTarget[0]).getDegrees()) < kTurnToleranceDeg)
                    .withTimeout(3.0),

            // Stop
            drivetrain.runOnce(() -> drivetrain.setControl(new SwerveRequest.SwerveDriveBrake()))
        );
    }

    /** Blue faces 0 deg (toward red wall), red faces 180 deg (toward blue wall). */
    private static Rotation2d awayFromOurWall() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? Rotation2d.k180deg
                : Rotation2d.kZero;
    }
}
