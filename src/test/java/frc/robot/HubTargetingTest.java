package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

class HubTargetingTest {

    @BeforeAll
    static void startHal() {
        HAL.initialize(500, 0);
    }

    @AfterEach
    void resetAlliance() {
        setAlliance(AllianceStationID.Blue1);
    }

    private static void setAlliance(AllianceStationID station) {
        DriverStationSim.setAllianceStationId(station);
        DriverStationSim.notifyNewData();
    }

    @Test
    void hubCentersMatchTheFieldDrawing() {
        // From the 2026 field layout: blue hub ~182.1 in from the blue wall, red hub ~469.1 in, both at mid-field (158.8 in)
        assertEquals(Units.inchesToMeters(182.107), HubTargeting.kBlueHubCenter.getX(), 0.01);
        assertEquals(Units.inchesToMeters(469.115), HubTargeting.kRedHubCenter.getX(), 0.01);
        assertEquals(Units.inchesToMeters(158.844), HubTargeting.kBlueHubCenter.getY(), 0.01);
        assertEquals(Units.inchesToMeters(158.844), HubTargeting.kRedHubCenter.getY(), 0.01);
    }

    @Test
    void blueAimsAtBlueHubAndRedAimsAtRedHub() {
        setAlliance(AllianceStationID.Blue1);
        assertEquals(HubTargeting.kBlueHubCenter, HubTargeting.hubCenter());

        setAlliance(AllianceStationID.Red2);
        assertEquals(HubTargeting.kRedHubCenter, HubTargeting.hubCenter());
    }

    @Test
    void distanceIsMeasuredToOurHubCenter() {
        setAlliance(AllianceStationID.Blue1);
        // 2 m straight toward the blue wall from the blue hub center
        Pose2d robot = new Pose2d(HubTargeting.kBlueHubCenter.getX() - 2.0, HubTargeting.kBlueHubCenter.getY(), Rotation2d.kZero);
        assertEquals(2.0, HubTargeting.distanceToHubMeters(robot), 1e-9);
        assertEquals(Units.metersToFeet(2.0), HubTargeting.distanceToHubFeet(robot), 1e-9);
    }

    @Test
    void linedUpOnlyWhenFacingTheHub() {
        setAlliance(AllianceStationID.Blue1);
        double x = HubTargeting.kBlueHubCenter.getX() - 2.0;
        double y = HubTargeting.kBlueHubCenter.getY();

        // Blue robot between its wall and hub, facing +X (toward the hub) is lined up
        assertTrue(HubTargeting.isLinedUp(new Pose2d(x, y, Rotation2d.kZero)));
        // Inside the 3 degree tolerance
        assertTrue(HubTargeting.isLinedUp(new Pose2d(x, y, Rotation2d.fromDegrees(2.5))));
        // Outside it
        assertFalse(HubTargeting.isLinedUp(new Pose2d(x, y, Rotation2d.fromDegrees(5))));
        // Facing the wrong way
        assertFalse(HubTargeting.isLinedUp(new Pose2d(x, y, Rotation2d.k180deg)));
    }

    @Test
    void redRobotFacesRedHub() {
        setAlliance(AllianceStationID.Red1);
        // Red robot 2 m from the red hub on the red-wall side: the hub is toward -X, so it must face 180 degrees
        Pose2d robot = new Pose2d(HubTargeting.kRedHubCenter.getX() + 2.0, HubTargeting.kRedHubCenter.getY(), Rotation2d.k180deg);
        assertEquals(180.0, Math.abs(HubTargeting.headingToFaceHub(robot).getDegrees()), 1e-6);
        assertTrue(HubTargeting.isLinedUp(robot));
    }

    @Test
    void scoringZoneIsBetweenOurWallAndOurTrench() {
        double midY = Units.inchesToMeters(158.8);

        setAlliance(AllianceStationID.Blue1);
        assertTrue(HubTargeting.isInScoringZone(new Pose2d(2.0, midY, Rotation2d.kZero)));      // near the blue wall
        assertFalse(HubTargeting.isInScoringZone(new Pose2d(8.27, midY, Rotation2d.kZero)));    // field center
        assertFalse(HubTargeting.isInScoringZone(new Pose2d(14.5, midY, Rotation2d.kZero)));    // red's zone

        setAlliance(AllianceStationID.Red3);
        assertTrue(HubTargeting.isInScoringZone(new Pose2d(14.5, midY, Rotation2d.kZero)));     // near the red wall
        assertFalse(HubTargeting.isInScoringZone(new Pose2d(8.27, midY, Rotation2d.kZero)));
        assertFalse(HubTargeting.isInScoringZone(new Pose2d(2.0, midY, Rotation2d.kZero)));
    }
}
