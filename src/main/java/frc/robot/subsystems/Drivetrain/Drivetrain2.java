package frc.robot.subsystems.Drivetrain;

import frc.robot.Constants;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain2 extends SubsystemBase{
    private final SwerveDriveWheel2 frontLeft = new SwerveDriveWheel2(
        Constants.DriveConstants.kFrontLeftDriveMotorPort,
        Constants.DriveConstants.kFrontLeftTurningMotorPort,
        Constants.DriveConstants.kFrontLeftTurningEncoderPort,
        Constants.DriveConstants.kFrontLeftTurningEncoderOffsetDeg,
        Constants.DriveConstants.kFrontLeftTurningEncoderReversed);

    private final SwerveDriveWheel2 frontRight = new SwerveDriveWheel2(
        Constants.DriveConstants.kFrontRightDriveMotorPort,
        Constants.DriveConstants.kFrontRightTurningMotorPort,
        Constants.DriveConstants.kFrontRightTurningEncoderPort,
        Constants.DriveConstants.kFrontRightTurningEncoderOffsetDeg,
        Constants.DriveConstants.kFrontRightTurningEncoderReversed);

    private final SwerveDriveWheel2 backLeft = new SwerveDriveWheel2(
        Constants.DriveConstants.kBackLeftDriveMotorPort,
        Constants.DriveConstants.kBackLeftTurningMotorPort,
        Constants.DriveConstants.kBackLeftTurningEncoderPort,
        Constants.DriveConstants.kBackLeftTurningEncoderOffsetDeg,
        Constants.DriveConstants.kBackLeftTurningEncoderReversed);

    private final SwerveDriveWheel2 backRight = new SwerveDriveWheel2(
        Constants.DriveConstants.kBackRightDriveMotorPort,
        Constants.DriveConstants.kBackRightTurningMotorPort,
        Constants.DriveConstants.kBackRightTurningEncoderPort,
        Constants.DriveConstants.kBackRightTurningEncoderOffsetDeg,
        Constants.DriveConstants.kBackRightTurningEncoderReversed);

    final WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.DriveConstants.pigeon);
    /* private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, 
    gyro.getRotation2d(), ); */


    public Drivetrain2() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    
    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /* public Pose2d getPose() {
        return odometer.getPoseMeters();
    } */

    /* public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(gyro.getRotation2d(), pose);
    } */

    public void stopWheels() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


}
