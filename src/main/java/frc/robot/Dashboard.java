package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Vision;

/**
 * Everything the drive team sees on the Elastic dashboard is published from here.
 *
 * The layout lives in src/main/deploy/elastic-layout.json. In Elastic, use
 * File > Load Layout From Robot (Ctrl+D) to load it.
 *
 * If you rename an entry here, rename its "topic" in elastic-layout.json too,
 * or that widget will go blank.
 *
 * Entries are grouped by folder:
 *   Auto/   - picking and watching the autonomous routine
 *   Match/  - match time and battery
 *   Aim/    - distance and angle to our hub, scoring zone
 *   Robot/  - mechanism status (index, intake arm)
 *   Vision/ - Limelight status
 *   Drive/  - swerve modules
 *   Debug/  - command scheduler and other pit-only info
 */
public class Dashboard {

    //---------- Elastic tab names (must match elastic-layout.json) ----------//
    public static final String kAutoTab = "Autonomous";
    public static final String kTeleopTab = "Teleoperated";
    public static final String kPitTab = "Pit";

    private final CommandSwerveDrivetrain drivetrain;
    private final Index index;
    private final IntakeArm intakeArm;
    private final Vision vision;

    /* Field map showing where the robot thinks it is */
    private final Field2d field = new Field2d();

    /* Latest swerve state, read once per loop and shared with the swerve widget */
    private SwerveDriveState swerveState;

    /* Elastic switches to whichever tab name is written here */
    private final StringPublisher selectedTab = NetworkTableInstance.getDefault()
            .getStringTopic("/Elastic/SelectedTab")
            .publish(PubSubOption.keepDuplicates(true));

    public Dashboard(CommandSwerveDrivetrain drivetrain, Index index, IntakeArm intakeArm,
            Vision vision, SendableChooser<Command> autoChooser) {
        this.drivetrain = drivetrain;
        this.index = index;
        this.intakeArm = intakeArm;
        this.vision = vision;
        this.swerveState = drivetrain.getState();

        SmartDashboard.putData("Auto/Auto Chooser", autoChooser);
        setAutoState("Waiting");

        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Drive/Swerve", this::buildSwerveWidget);
        SmartDashboard.putData("Debug/Scheduler", CommandScheduler.getInstance());
    }

    /** Call once per loop from Robot.robotPeriodic(). */
    public void update() {
        swerveState = drivetrain.getState();
        field.setRobotPose(swerveState.Pose);

        //---------- Match ----------//
        SmartDashboard.putNumber("Match/Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Match/Battery Voltage", RobotController.getBatteryVoltage());

        //---------- Robot mechanisms ----------//
        SmartDashboard.putBoolean("Robot/Index Running", index.isRunning());
        SmartDashboard.putString("Robot/Index Last Command", index.getLastCaller());
        SmartDashboard.putBoolean("Robot/Intake Arm Up", intakeArm.getLimitUp());
        SmartDashboard.putBoolean("Robot/Intake Arm Down", intakeArm.getLimitDown());

        //---------- Aiming at our hub (see HubTargeting.java) ----------//
        var pose = swerveState.Pose;
        SmartDashboard.putNumber("Aim/Distance to Hub (ft)",
                Math.round(HubTargeting.distanceToHubFeet(pose) * 10) / 10.0); // 1 decimal place
        SmartDashboard.putNumber("Aim/Angle Error (deg)",
                Math.round(HubTargeting.aimErrorDegrees(pose) * 10) / 10.0);
        SmartDashboard.putBoolean("Aim/Lined Up", HubTargeting.isLinedUp(pose));
        SmartDashboard.putBoolean("Aim/In Scoring Zone", HubTargeting.isInScoringZone(pose));
        field.getObject("Our Hub").setPose(new Pose2d(HubTargeting.hubCenter(), Rotation2d.kZero));

        //---------- Vision ----------//
        SmartDashboard.putBoolean("Vision/Has Target", vision.hasTarget());
        SmartDashboard.putNumber("Vision/Tag Count", vision.getTagCount());
        SmartDashboard.putBoolean("Vision/Pose Accepted", vision.isPoseAccepted());
    }

    /** Shows what the autonomous routine is doing (Waiting / Running / Done). */
    public static void setAutoState(String state) {
        SmartDashboard.putString("Auto/State", state);
    }

    /** Switches Elastic to the named tab. Use the kAutoTab / kTeleopTab / kPitTab constants. */
    public void selectTab(String tabName) {
        selectedTab.set(tabName);
    }

    /* Publishes the four module angles and speeds in the format Elastic's Swerve Drive widget expects */
    private void buildSwerveWidget(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        // CTRE module order: 0 = Front Left, 1 = Front Right, 2 = Back Left, 3 = Back Right
        String[] names = {"Front Left", "Front Right", "Back Left", "Back Right"};
        for (int i = 0; i < names.length; i++) {
            final int module = i;
            builder.addDoubleProperty(names[i] + " Angle",
                    () -> hasModuleStates() ? swerveState.ModuleStates[module].angle.getRadians() : 0, null);
            builder.addDoubleProperty(names[i] + " Velocity",
                    () -> hasModuleStates() ? swerveState.ModuleStates[module].speedMetersPerSecond : 0, null);
        }
        builder.addDoubleProperty("Robot Angle",
                () -> swerveState.Pose.getRotation().getRadians(), null);
    }

    /* Module states are empty until the drivetrain's first odometry update */
    private boolean hasModuleStates() {
        return swerveState.ModuleStates != null && swerveState.ModuleStates.length == 4;
    }
}
