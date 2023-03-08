package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain2;

public class MakeRoboGo extends CommandBase{
    private final Drivetrain2 m_drivetrain;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private double xSpeed, ySpeed, turningSpeed;
    private boolean fieldRelative;


    public MakeRoboGo(Drivetrain2 m_drivetrain, double xSpeed, double ySpeed, double turningSpeed, boolean fieldRelative) {
            //Create SlewRateLimiters for x, y, and turning speeds
            xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
            yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
            turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
            this.xSpeed = xSpeed;
            this.ySpeed=ySpeed;
            this.turningSpeed = turningSpeed;
            this.m_drivetrain = m_drivetrain;
            this.fieldRelative = fieldRelative;
            
            addRequirements(m_drivetrain);
    }


  

    @Override
    public void initialize() {}


    @Override
    public void execute() {

            // Get the x speed. We are inverting this because Xbox controllers return negative values when we push forward.
            xSpeed = -xLimiter.calculate(MathUtil.applyDeadband(xSpeed, 0.02)) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

            // Get the y speed or sideways/strafe speed. We are inverting this because we want a positive value when we pull to 
            //the left. Xbox controllers return positive values when you pull to the right by default.
            ySpeed = -yLimiter.calculate(MathUtil.applyDeadband(ySpeed, 0.02)) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;


            // Get the rate of angular rotation. We are inverting this because we want a positive value when we pull to the left 
            //(remember, CCW is positive in mathematics). Xbox controllers return positive values when you pull to the right by default.
            turningSpeed = -turningLimiter.calculate(MathUtil.applyDeadband(turningSpeed, 0.02)) * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            SmartDashboard.putNumber("xSpeed with Limiter", xSpeed); //should be meters per second
            SmartDashboard.putNumber("ySpeed with Limiter", ySpeed); //should be meters per second
            SmartDashboard.putNumber("turningSpeed with Limiter", turningSpeed); //should be radians per second

            //Call drive method with updated x, y, and turning speeds
            m_drivetrain.drive(xSpeed, ySpeed, turningSpeed, fieldRelative);

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
