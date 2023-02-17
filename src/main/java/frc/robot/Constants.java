package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


public final class Constants {
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.14;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        //public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        //public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        //public static final double kTurningEncoderRot2Deg = kTurningMotorGearRatio * 360.0;
        //public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        //public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        //public static final double kTurningEncoderRPM2DegPerSec = kTurningEncoderRot2Deg / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {
        public static final double kPhysicalMaxSpeedMetersPerSecond = 4;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 1 * 2 * Math.PI;

        public static final double kTrackWidth = Units.inchesToMeters(22);
            // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22);
            // Distance between front and back wheels
        
        //Changed this to what I think is correct, not the tutorial
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
        public static final int kFrontLeftDriveMotorPort = 7;
        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 1;
        public static final int kBackRightDriveMotorPort = 3;
    
        public static final int kFrontLeftTurningMotorPort = 8;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kFrontRightTurningMotorPort = 2;
        public static final int kBackRightTurningMotorPort = 4;
    
        public static final int kFrontLeftTurningEncoderPort = 16;
        public static final int kBackLeftTurningEncoderPort = 15;
        public static final int kFrontRightTurningEncoderPort = 13;
        public static final int kBackRightTurningEncoderPort = 14;

        public static final double kFrontLeftTurningEncoderOffsetDeg = 0.0;
        public static final double kBackLeftTurningEncoderOffsetDeg = 0.0;
        public static final double kFrontRightTurningEncoderOffsetDeg = 0.0;
        public static final double kBackRightTurningEncoderOffsetDeg = 0.0;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1.0;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.0;

        public static int pigeon = 21;

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kArmControllerPort = 1;

/*         public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverFieldOrientedButtonIdx = 4; */

        public static final double kDeadband = 0.05;
    }
    
}
