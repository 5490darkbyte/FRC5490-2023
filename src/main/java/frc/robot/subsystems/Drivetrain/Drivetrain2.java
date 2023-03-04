package frc.robot.subsystems.Drivetrain;

import frc.robot.Constants;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax.IdleMode;

//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain2 extends SubsystemBase{
    private final SwerveDriveWheel2 frontLeft = new SwerveDriveWheel2(
        Constants.DriveConstants.kFrontLeftDriveMotorPort,
        Constants.DriveConstants.kFrontLeftTurningMotorPort,
        Constants.DriveConstants.kFrontLeftTurningEncoderPort,
        Constants.DriveConstants.kFrontLeftTurningEncoderOffsetDeg,
        Constants.DriveConstants.kFrontLeftDriveEncoderReversed,
        Constants.DriveConstants.kFrontLeftTurningEncoderReversed,
        Constants.DriveConstants.kfrontLeftOffset);

    private final SwerveDriveWheel2 frontRight = new SwerveDriveWheel2(
        Constants.DriveConstants.kFrontRightDriveMotorPort,
        Constants.DriveConstants.kFrontRightTurningMotorPort,
        Constants.DriveConstants.kFrontRightTurningEncoderPort,
        Constants.DriveConstants.kFrontRightTurningEncoderOffsetDeg,
        Constants.DriveConstants.kFrontRightDriveEncoderReversed,
        Constants.DriveConstants.kFrontRightTurningEncoderReversed,
        Constants.DriveConstants.kfrontRightOffset);

    private final SwerveDriveWheel2 backLeft = new SwerveDriveWheel2(
        Constants.DriveConstants.kBackLeftDriveMotorPort,
        Constants.DriveConstants.kBackLeftTurningMotorPort,
        Constants.DriveConstants.kBackLeftTurningEncoderPort,
        Constants.DriveConstants.kBackLeftTurningEncoderOffsetDeg,
        Constants.DriveConstants.kBackLeftDriveEncoderReversed,
        Constants.DriveConstants.kBackLeftTurningEncoderReversed,
        Constants.DriveConstants.kbackLeftOffset);

    private final SwerveDriveWheel2 backRight = new SwerveDriveWheel2(
        Constants.DriveConstants.kBackRightDriveMotorPort,
        Constants.DriveConstants.kBackRightTurningMotorPort,
        Constants.DriveConstants.kBackRightTurningEncoderPort,
        Constants.DriveConstants.kBackRightTurningEncoderOffsetDeg,
        Constants.DriveConstants.kBackRightDriveEncoderReversed,
        Constants.DriveConstants.kBackRightTurningEncoderReversed,
        Constants.DriveConstants.kbackRightOffset);

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

    public void resetEncodersToAbsolutes(){
        frontRight.resetAbsEncoder();
        backRight.resetAbsEncoder();
        backLeft.resetAbsEncoder();
        frontLeft.resetAbsEncoder();
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

    public double turningPosRadToDegrees(double measured){
        return  (measured * 180/Math.PI);
      } 

    public void setDriveMotors(double desiredSpeeds) {
        //tell the motor controllers how fast the motors should spin 
        frontRight.setDriveMotor(desiredSpeeds);
        backRight.setDriveMotor(desiredSpeeds);
        backLeft.setDriveMotor(desiredSpeeds);
        frontLeft.setDriveMotor(desiredSpeeds);
      }


        //Use PID controllers to turn wheels to certain degrees
        public boolean turnToDegrees(double[] desiredDegree){
            frontRight.setTurnMotor(frontRight.turningPidController.calculate(Constants.DriveConstants.kfrontRightOffset - turningPosRadToDegrees(frontRight.getAbsoluteTurningPosition()), desiredDegree[0])); 
            backRight.setTurnMotor(backRight.turningPidController.calculate(Constants.DriveConstants.kbackRightOffset - turningPosRadToDegrees(backRight.getAbsoluteTurningPosition()), desiredDegree[1])); 
            backLeft.setTurnMotor(backLeft.turningPidController.calculate(Constants.DriveConstants.kbackLeftOffset - turningPosRadToDegrees(backLeft.getAbsoluteTurningPosition()), desiredDegree[2])); 
            frontLeft.setTurnMotor(frontLeft.turningPidController.calculate(Constants.DriveConstants.kfrontLeftOffset - turningPosRadToDegrees(frontLeft.getAbsoluteTurningPosition()), desiredDegree[3])); 
            if (frontRight.turningPidController.atSetpoint())
              frontRight.setTurnMotor(0);
            if (backRight.turningPidController.atSetpoint())
              backRight.setTurnMotor(0);
            if (backLeft.turningPidController.atSetpoint())
              backLeft.setTurnMotor(0);
            if (frontLeft.turningPidController.atSetpoint())
              frontLeft.setTurnMotor(0);
      
            if (frontRight.turningPidController.atSetpoint() && backRight.turningPidController.atSetpoint() && backLeft.turningPidController.atSetpoint())
              return true;
            else
              return false;
          }
      

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
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

@Override
    public void periodic() {
        SmartDashboard.putNumber("Relative Position 8",frontLeft.getTurningPosition());
        SmartDashboard.putNumber("Relative Position 2",frontRight.getTurningPosition());
        SmartDashboard.putNumber("Relative Position 6",backLeft.getTurningPosition());
        SmartDashboard.putNumber("Relative Position 4",backRight.getTurningPosition());

        SmartDashboard.putNumber("Turning Encoder1 AbsolutePos", frontRight.getAbsoluteTurningPosition());
        SmartDashboard.putNumber("Turning Encoder2 AbsolutePos", backRight.getAbsoluteTurningPosition());
        SmartDashboard.putNumber("Turning Encoder3 AbsolutePos", backLeft.getAbsoluteTurningPosition());
        SmartDashboard.putNumber("Turning Encoder4 AbsolutePos", frontLeft.getAbsoluteTurningPosition());



        
    }

}
