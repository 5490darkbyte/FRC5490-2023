package frc.robot.subsystems.Drivetrain;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.mathFunctions;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class SwerveDriveWheel2 {
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    private final CANCoder absoluteEncoder;

    //private final PIDController turningPidController;
    private final SparkMaxPIDController turningPidController2;

    private final boolean turningEncoderReversedBool;
    private final double turningEncoderOffsetRad;

    public SwerveDriveWheel2(int driveMotorId, int turningMotorId, int turningEncoderId, double turningEncoderOffset, boolean driveEncoderReversed, boolean turningEncoderReversed) {

        turningEncoderOffsetRad = turningEncoderOffset * (180/Math.PI);
        turningEncoderReversedBool = turningEncoderReversed;

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveEncoderReversed);
        turningMotor.setInverted(turningEncoderReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        absoluteEncoder = new CANCoder(turningEncoderId);

        turningEncoder.setPositionConversionFactor(0.5);
        driveEncoder.setPositionConversionFactor(1);



        //turningPidController = new PIDController(ModuleConstants.kPTurning, 0.0, 0.0);

        turningPidController2 = turningMotor.getPIDController();
        turningPidController2.setP(ModuleConstants.kPTurning);

        //turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        turningPidController2.setPositionPIDWrappingEnabled(true);
        turningPidController2.setPositionPIDWrappingMaxInput(10*Math.PI);
        turningPidController2.setPositionPIDWrappingMinInput(8*Math.PI);


            resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    } 

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }
    
    public double getAbsoluteTurningPosition(){
        return absoluteEncoder.getAbsolutePosition();
    }

     public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    } 

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getTurningDeg() {
        double angle = turningEncoder.getPosition();
        angle *= (180/Math.PI);
        angle -= turningEncoderOffsetRad;
        return angle * (turningEncoderReversedBool ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }
        // state = SwerveModuleState.optimize(state, new Rotation2d(getTurningPosition())); //getState().angle
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        
        turningPidController2.setReference(state.angle.getRadians(), ControlType.kPosition);
        //turningMotor.set(turningPidController.calculate(getTurningPosition(), 3.14));//
        //mathFunctions.turningSpeedCalculate(state.angle.getRadians()/0.02));
        SmartDashboard.putNumber("Desired Position " + turningMotor.getDeviceId(), state.angle.getRadians());
    }


    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
