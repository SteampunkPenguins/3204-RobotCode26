package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    
    //---------- Maximum Current of SparkMax and Falcon Motors ----------//
    public static final int kMaxCurrent = 40;
    
    public static class Intake{
        //ID of the Intake Motor
        public static final int kIntakeMotor = 21;
    }
    
    public static class Shooter{
        //IDs of the Shooter Motors
        public static final int kShooterMotor = 17;
        public static final int kShooterNeckMotor = 18;
    }

    public static class IntakeArm{
        //ID of the Intake Arm Motor
        public static final int kIntakeArmMotor = 22;
       
        //IDs of the Limit Switches
        public static final int kArmPassive = 0;
        public static final int kArmActive = 1;
    }

    public static class Index{
        //ID of the Index Motor
        public static final int kIntakeIndexMotor = 20;
    }

    public static class Vision{
        //Must match the name set in the Limelight web UI (http://limelight.local:5801)
        public static final String kLimelightName = "limelight";

        //Ignore vision while the robot spins faster than this
        public static final double kMaxYawRateDegPerSec = 720;

        //How much to trust vision X/Y (meters). Bigger = trust it less.
        public static final double kXYStdDev = 0.7;

        //---------- Camera mount, measured from the CENTER of the robot on the floor ----------//
        //Sent to the Limelight at startup (overrides what's typed in the Limelight web UI).
        //TODO: measure these on the real robot
        public static final double kCamForwardMeters = Units.inchesToMeters(12); //+ = toward the front
        public static final double kCamSideMeters = 0;                           //+ = toward the left
        public static final double kCamUpMeters = Units.inchesToMeters(5);       //height above the floor
        public static final double kCamRollDeg = 0;
        public static final double kCamPitchDeg = 15;                            //+ = tilted up
        public static final double kCamYawDeg = 0;                               //+ = turned left

        //---------- Simulation only ----------//
        //Field of view (Limelight 4 values) and how far away the fake camera can see a tag
        public static final double kSimHorizontalFovDeg = 82;
        public static final double kSimVerticalFovDeg = 56.2;
        public static final double kSimMaxTagDistanceMeters = 5.0;
    }
}
