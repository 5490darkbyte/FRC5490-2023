package frc.robot.subsystems.Drivetrain;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class SwerveDriveWheel2 {
    //radius is 2 inches in meters
    private static final double kWheelRadius = 0.0508;
   
    //CANCoder counts per revolution (41 times 10:1 gearbox)
    private static final int kDrivingEncoderResolution = 420;
    private static final int kAbsEncoderResolution = 4096;
    
    // declare variables for motor controllers
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    // declare variables for two relative encoders and one absolute CANCoder
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    private final CANCoder absoluteEncoder;

    // declare PID for turning wheel direction
    private final SparkMaxPIDController turningPidController2;

    
    // declare PID for driving wheel distance
    private final SparkMaxPIDController drivingPidController2;


    // declare boolean to identify if the absolute encoder is inverted
    private final boolean turningEncoderReversed;

    public SwerveDriveWheel2(int driveMotorId, int turningMotorId, int turningAbsEncoderId,
            boolean driveEncoderReversed, boolean turningEncoderReversed) {
       
        // Initialize motor controllers
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        // Set whether motor controllers should be inverted
        driveMotor.setInverted(driveEncoderReversed);
        turningMotor.setInverted(turningEncoderReversed);
        this.turningEncoderReversed = turningEncoderReversed;

        // Intitialize relative encoders
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        // Initialize absolute CANCoder
        absoluteEncoder = new CANCoder(turningAbsEncoderId);

        // Set Abs encoder position to absolute
        resetAbsEncoder();

        // Initialize sparkmax PIDs for manual driving
        drivingPidController2 = driveMotor.getPIDController();
        drivingPidController2.setP(DriveConstants.kPDriving);
        turningPidController2 = turningMotor.getPIDController();
        turningPidController2.setP(DriveConstants.kPTurning);


        //set output ranges for pid controllers
        turningPidController2.setOutputRange(-1, 1);
        drivingPidController2.setOutputRange(-1, 1);;

        // double check these values
        turningPidController2.setPositionPIDWrappingEnabled(true);
        turningPidController2.setPositionPIDWrappingMaxInput(10 * Math.PI);
        turningPidController2.setPositionPIDWrappingMinInput(8 * Math.PI);

        // double check these values
        turningPidController2.setPositionPIDWrappingEnabled(true);
        turningPidController2.setPositionPIDWrappingMaxInput(10 * Math.PI);
        turningPidController2.setPositionPIDWrappingMinInput(8 * Math.PI);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder resolution.
        driveEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius / kDrivingEncoderResolution);

         // Set the distance (in this case, angle) in radians per second for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the encoder resolution.
         absoluteEncoder.configFeedbackCoefficient(2 * Math.PI / kAbsEncoderResolution, "radians", SensorTimeBase.PerSecond);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(absoluteEncoder.getPosition()));
      }


    // getter for driving relative encoder value (radians)
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }


    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(absoluteEncoder.getPosition()));
      }


    // getter for turning relative encoder value (radians)
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    // getter for turning absolute CANCoder value (radians)
    public double getAbsoluteTurningPosition() {
        return absoluteEncoder.getAbsolutePosition();
    }

    // current velocity of driving relative encoder in RPM
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    // current velocity of turning relative encoder in RPM
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    // converts current relative turning encoder position to degrees
    public double getTurningDeg() {
        double angle = turningEncoder.getPosition();
        angle *= (180 / Math.PI);
        return angle * (turningEncoderReversed ? -1.0 : 1.0);
    }

    // run turning motor at desired percent output
    public void setTurnMotor(double turnSpeed) {
        turningMotor.set(turnSpeed);
    }

    // run drive motor at desired percent output
    public void setDriveMotor(double driveSpeed) {
        driveMotor.set(driveSpeed);
    }

    // set current relative encoder position to 0
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
    }

    // Make absolute turning encoder match the magnet's absolute
    public void resetAbsEncoder() {
        absoluteEncoder.setPositionToAbsolute();

    }

    public boolean wheelisForward(){
        if (absoluteEncoder.getAbsolutePosition() > -.1 && absoluteEncoder.getAbsolutePosition() < .1)
        return true;
        else
        return false;
    }

    // set motors to brake mode or coast mode
    public void enableMotors(IdleMode mode) {
        turningMotor.setIdleMode(mode);
        driveMotor.setIdleMode(mode);
    }

 
    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(absoluteEncoder.getPosition()));

        // Calculate the drive output from the drive PID controller.
        drivingPidController2.setReference(driveEncoder.getVelocity(), ControlType.kVelocity);

        // Calculate the turning motor output from the turning PID controller.
        turningPidController2.setReference(turningEncoder.getPosition(), ControlType.kPosition);

        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(state.angle.getRadians());
       

    }

    // Set both wheels to 0
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
