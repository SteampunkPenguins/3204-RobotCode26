// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Index;
import frc.robot.sim.LimelightSim;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* Only created when running in the simulator */
    private LimelightSim m_limelightSim;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        // Lets Elastic download the dashboard layout from the robot:
        // File > Load Layout From Robot (Ctrl+D), then pick elastic-layout.json
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        // Start on the Autonomous tab so the drive team picks an auto first
        m_robotContainer.dashboard.selectTab(Dashboard.kAutoTab);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        m_robotContainer.dashboard.update();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        DriverStation.reportError("Report::autonomousInit: Started!", isAutonomous());

        //Calls the Selected Manual Auto
        // m_autonomousCommand = m_robotContainer.getSelectedAuto();
        
        //Calls the Selected Path Planner Auto
        m_autonomousCommand = m_robotContainer.getPathPlannerAuto();

        m_robotContainer.dashboard.selectTab(Dashboard.kAutoTab);

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
            Dashboard.setAutoState("Running");
        } else {
            Dashboard.setAutoState("No auto selected");
        }

    }

    @Override
    public void autonomousPeriodic() {
        //DriverStation.reportError("Report::autonomousPeriodic: Periodic()", isAutonomous());
    }

    @Override
    public void autonomousExit() {
        DriverStation.reportWarning("Report::autonomousExit: Done", isAutonomous());
        Dashboard.setAutoState("Done");
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        
        Shooter.getInstance().spinShooterZero();
        Index.getInstance().spinIndexZero();

        // Drivers only need the Teleoperated tab from here on
        m_robotContainer.dashboard.selectTab(Dashboard.kTeleopTab);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationInit() {
        m_limelightSim = new LimelightSim();
    }

    @Override
    public void simulationPeriodic() {
        // Fake Limelight: publishes what the camera would see from the sim robot's position
        m_limelightSim.update(m_robotContainer.m_robotDrive.getState().Pose);
    }
}
