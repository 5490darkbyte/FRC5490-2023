package frc.robot.subsystems.Drivetrain;

import frc.robot.Constants;
import frc.robot.RobotMap;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;


public class Drivetrain2 extends SubsystemBase{
    //Create an object for each wheel
    private final SwerveDriveWheel2 frontLeft = new SwerveDriveWheel2(
        Constants.DriveConstants.kFrontLeftDriveMotorPort,
        Constants.DriveConstants.kFrontLeftTurningMotorPort,
        Constants.DriveConstants.kFrontLeftTurningEncoderPort,
        Constants.DriveConstants.kFrontLeftDriveEncoderReversed,
        Constants.DriveConstants.kFrontLeftTurningEncoderReversed);

    private final SwerveDriveWheel2 frontRight = new SwerveDriveWheel2(
        Constants.DriveConstants.kFrontRightDriveMotorPort,
        Constants.DriveConstants.kFrontRightTurningMotorPort,
        Constants.DriveConstants.kFrontRightTurningEncoderPort,
        Constants.DriveConstants.kFrontRightDriveEncoderReversed,
        Constants.DriveConstants.kFrontRightTurningEncoderReversed);

    private final SwerveDriveWheel2 backLeft = new SwerveDriveWheel2(
        Constants.DriveConstants.kBackLeftDriveMotorPort,
        Constants.DriveConstants.kBackLeftTurningMotorPort,
        Constants.DriveConstants.kBackLeftTurningEncoderPort,
        Constants.DriveConstants.kBackLeftDriveEncoderReversed,
        Constants.DriveConstants.kBackLeftTurningEncoderReversed);

    private final SwerveDriveWheel2 backRight = new SwerveDriveWheel2(
        Constants.DriveConstants.kBackRightDriveMotorPort,
        Constants.DriveConstants.kBackRightTurningMotorPort,
        Constants.DriveConstants.kBackRightTurningEncoderPort,
        Constants.DriveConstants.kBackRightDriveEncoderReversed,
        Constants.DriveConstants.kBackRightTurningEncoderReversed);

    //Create Gyro Opbject
    final WPI_Pigeon2 gyro = new WPI_Pigeon2(RobotMap.pigeon);

    private final double distanceToWheel = Units.inchesToMeters(11);
    //Location for the swerve drive modules relative to robot center
    Translation2d frontRightLocation = new Translation2d(distanceToWheel, -distanceToWheel);
    Translation2d backRightLocation = new Translation2d(-distanceToWheel, -distanceToWheel);
    Translation2d backLeftLocation = new Translation2d(-distanceToWheel, distanceToWheel);
    Translation2d frontLeftLocation = new Translation2d(distanceToWheel, distanceToWheel);

    // Creating kinematics object using module locations
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontRightLocation, backRightLocation, backLeftLocation, frontLeftLocation);

    //Creating odometry object with kinematics, location of gyro, current swerve module positions, and starting location on field
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(),  
    new SwerveModulePosition[] {frontRight.getPosition(), backRight.getPosition(), backLeft.getPosition(), frontLeft.getPosition()}, 
    new Pose2d(0.0, 0.0, new Rotation2d()));

    

    public Drivetrain2() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
      SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()));
      
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);

      setSwerveModuleStates(swerveModuleStates);
    }
  

    //reset gyro at startup
    public void zeroHeading() {
        gyro.reset();
    }

    //access gyro location
    public double getHeading() {
      return Math.IEEEremainder(-gyro.getAngle(), 360);
  }

    //Updates the field relative position of the robot.
    public void updateOdometry() {
      odometry.update(gyro.getRotation2d(), getSwerveModulePositions());
    }


    public Pose2d getPose() {
      return odometry.getPoseMeters();
  }


  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), pose);
}

    //set all wheel turning absolute encoders to absolute values
    public void resetEncodersToAbsolutes(){
        frontRight.resetAbsEncoder();
        backRight.resetAbsEncoder();
        backLeft.resetAbsEncoder();
        frontLeft.resetAbsEncoder();
      }

      //set all relative encoders to 0 at current position
      public void resetRelativeEncoders(){
        frontRight.resetEncoders();
        backRight.resetEncoders();
        backLeft.resetEncoders();
        frontLeft.resetEncoders();
      }


      //set motors to brake mode or coast mode
      public void enableMotors(boolean on) {
        IdleMode mode;
        if (on) {
        mode = IdleMode.kBrake;
        } else {
        mode = IdleMode.kCoast;
        }
        frontRight.enableMotors(mode);
        backRight.enableMotors(mode);
        backLeft.enableMotors(mode);
        frontLeft.enableMotors(mode);
    }


    //set all wheel drive motors to a percent output
    public void setDriveMotors(double desiredSpeeds) {
        frontRight.setDriveMotor(desiredSpeeds);
        backRight.setDriveMotor(desiredSpeeds);
        backLeft.setDriveMotor(desiredSpeeds);
        frontLeft.setDriveMotor(desiredSpeeds);
      }

      //set all wheel turn motors to a percent output
      public void setTurnMotors(double desiredSpeeds) {
        frontRight.setTurnMotor(desiredSpeeds);
        backRight.setTurnMotor(desiredSpeeds);
        backLeft.setTurnMotor(desiredSpeeds);
        frontLeft.setTurnMotor(desiredSpeeds);
      }

      //returns the swervemodule positions as an array
      public SwerveModulePosition[] getSwerveModulePositions(){
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()};

        return swerveModulePositions;

}
      //Creates setpoint for each wheel
      public void setSwerveModuleStates(SwerveModuleState[] states){
          frontRight.setDesiredState(states[0]);
          backRight.setDesiredState(states[1]);
          backLeft.setDesiredState(states[2]);
          frontLeft.setDesiredState(states[3]);
      }   


 
    //stop all wheels from moving
    public void stopWheels() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    //Identify if wheels are facing forward
    public boolean WheelsAreFoward(){
      if (frontRight.wheelisForward()){
        frontRight.setTurnMotor(0);
      }

      if (backRight.wheelisForward()){
        backRight.setTurnMotor(0);
      }

      if (backLeft.wheelisForward()){
        backLeft.setTurnMotor(0);
      }

      if (frontLeft.wheelisForward()){
        frontLeft.setTurnMotor(0);
      }

      if (frontRight.wheelisForward() && backRight.wheelisForward() && backLeft.wheelisForward() && frontLeft.wheelisForward())
        return true;
      else
      return false;

    }

   

@Override
    public void periodic() {

      odometry.update(gyro.getRotation2d(), getSwerveModulePositions());

      SmartDashboard.putNumber("Robot Heading", getHeading());
      SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

      SmartDashboard.putNumber("Turning Encoder1 AbsolutePos", frontRight.getAbsoluteTurningPosition());
      SmartDashboard.putNumber("Turning Encoder2 AbsolutePos", backRight.getAbsoluteTurningPosition());
      SmartDashboard.putNumber("Turning Encoder3 AbsolutePos", backLeft.getAbsoluteTurningPosition());
      SmartDashboard.putNumber("Turning Encoder4 AbsolutePos", frontLeft.getAbsoluteTurningPosition());

  }

}
