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
//import com.revrobotics.RelativeEncoder;

public class SwerveDriveWheel2 {
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    //private final RelativeEncoder driveEncoder;
    private final CANCoder turningEncoder;

    private final PIDController turningPidController;

    private final boolean turningEncoderReversedBool;
    private final double turningEncoderOffsetRad;

    public SwerveDriveWheel2(int driveMotorId, int turningMotorId, int turningEncoderId, double turningEncoderOffset, boolean turningEncoderReversed) {

        turningEncoderOffsetRad = turningEncoderOffset * (180/Math.PI);
        turningEncoderReversedBool = turningEncoderReversed;

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        //driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningEncoderReversed);

        //driveEncoder = driveMotor.getEncoder();
        turningEncoder = new CANCoder(turningEncoderId);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0.0, 0.0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

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

    public double getTurningDeg() {
        double angle = turningEncoder.getPosition();
        angle *= (180/Math.PI);
        angle -= turningEncoderOffsetRad;
        return angle * (turningEncoderReversedBool ? -1.0 : 1.0);
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
        driveMotor.set(0);//state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        
        
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //mathFunctions.turningSpeedCalculate(state.angle.getRadians()/0.02));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
