package frc.robot.commands;

//timer library
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.Drivetrain2;


public class DriveForward extends CommandBase {
      //declare startTime variable
        private double startTime;
        private final Drivetrain2 SwerveDrive;
     
    
    public DriveForward(Drivetrain2 SwerveDrive) {
        this.SwerveDrive = SwerveDrive;
        addRequirements(SwerveDrive);
    }
    
    @Override
    public void initialize() {  
        //set starttime to time when autonomous mode is started
        startTime = Timer.getFPGATimestamp();

        //put motors in brake mode (true)
        SwerveDrive.enableMotors(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //set current time to time since robot turned on
        double time = Timer.getFPGATimestamp();

        //run speed motor at 20% speed for 3 seconds
        if (time - startTime < 3) {
            SwerveDrive.setDriveMotors(0.2);
            SwerveDrive.setTurnMotors(0);
        }
        else
            SwerveDrive.setDriveMotors(0);
            SwerveDrive.setTurnMotors(0);

        }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
      
}
