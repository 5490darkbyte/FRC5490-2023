package frc.robot.commands;


import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain2;

public class MakeRoboGo extends CommandBase{

    //RobotContainer m_container;

    private final Drivetrain2 m_drivetrain;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    //private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;



    public MakeRoboGo(Drivetrain2 m_drivetrain, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction) {
            this.m_drivetrain = m_drivetrain;
            this.xSpdFunction = xSpdFunction;
            this.ySpdFunction = ySpdFunction;
            this.turningSpdFunction = turningSpdFunction;
            this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
            this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
            this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
            
            addRequirements(m_drivetrain);
    }


    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > Constants.OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond; 
        
        //turningSpeed = 0.658;

        turningSpeed = /*turningLimiter.calculate(*/turningSpeed* Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds relative to field
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, m_drivetrain.getRotation2d());


        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        m_drivetrain.setModuleStates(moduleStates);

        SmartDashboard.putNumber("LeftStickX", xSpdFunction.get());
        SmartDashboard.putNumber("LeftStickY", ySpdFunction.get());
        SmartDashboard.putNumber("RightStickX", turningSpdFunction.get());

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stopWheels();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
