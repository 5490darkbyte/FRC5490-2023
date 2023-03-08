package frc.robot;
import edu.wpi.first.math.util.Units;


public final class Constants {

    public static final class DriveConstants {
        public static final double kPhysicalMaxSpeedMetersPerSecond = 3.66;
        //public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 1 * 2 * Math.PI;

        public static final double kTrackWidth = Units.inchesToMeters(22);
            // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22);
            // Distance between front and back wheels

        
        //CAN IDs
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

        public static final int kFrontLeftDriveEncoderPort = 30;
        public static final int kBackLeftDriveEncoderPort = 31;
        public static final int kFrontRightDriveEncoderPort = 32;
        public static final int kBackRightDriveEncoderPort = 33;

        //Invert encoders if necessary
        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;        

        //Max speeds and acceleration
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond - 1;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3.66;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = Math.PI;

        //kP for PID controllers
        //for moving wheels to facing forward
        public static final double kTurningPIDProportion = .04;
        //for lifting shoulder
        public static final double kShoulderPIDProportion = .003;
      
        //for swerve drive
        public static final double kPTurning = 0.085;
        //for swerve drive
        public static final double kPDriving = 0.003;
    }

    public static final class OIConstants {
        //CAN IDs
        public static final int kDriverControllerPort = 0;
        public static final int kArmControllerPort = 1;

        //XBox controller Button IDs
        public static final int kArmXButton = 3;
        public static final int kArmYButton = 4;

        //Joystick Deadband
        public static final double kDeadband = 0.07;
    }
    
}
