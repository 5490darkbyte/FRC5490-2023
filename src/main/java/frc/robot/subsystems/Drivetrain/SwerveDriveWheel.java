package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/* import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; */


public class SwerveDriveWheel
{

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    //private final RelativeEncoder driveEncoder;
    private final CANCoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder encoder;
    private final boolean encoderReversed;
    private final double encoderOffsetDeg;

    public SwerveDriveWheel(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
    int encoderId, double encoderOffset, boolean absoluteEncoderReversed) {

    this.encoderOffsetDeg = encoderOffset;
    this.encoderReversed = absoluteEncoderReversed;
    encoder = new CANCoder(encoderId);

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    //driveEncoder = driveMotor.getEncoder();
    turningEncoder = new CANCoder(driveMotorId);

    //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    //turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-180.0, 180.0);

    //resetEncoders();
    }

    public double getDrivePosition() {
        return 0;
        //return driveEncoder.getPosition();
    } 

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

     public double getDriveVelocity() {
        return 0;
        //return driveEncoder.getVelocity();
    } 

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getencoderDeg() {
        double angle = encoder.getPosition();
        angle -= encoderOffsetDeg;
        return angle * (encoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        //driveEncoder.setPosition(0);
        //turningEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getDegrees()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
 
}
