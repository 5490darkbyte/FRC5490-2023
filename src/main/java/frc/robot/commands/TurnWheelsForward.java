
package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.Drivetrain2;


public class TurnWheelsForward extends CommandBase {
    private final Drivetrain2 SwerveDrive;

    private boolean finished = false;


    public TurnWheelsForward(Drivetrain2 SwerveDrive) {
        this.SwerveDrive = SwerveDrive;
        addRequirements(SwerveDrive);
    }

    @Override
    public void initialize() {
        finished = false;
        SwerveDrive.resetEncodersToAbsolutes();
        
        //put motors in brake mode
        SwerveDrive.enableMotors(true);

        //Stop drive motors
        SwerveDrive.setDriveMotors(0);

    
    }

    @Override
    public void execute() {
       //Turn all wheels to straight forward
       finished = SwerveDrive.turnToDegrees(new double[4]);
    }

    @Override
    public void end(boolean interrupted) {}


    @Override
    public boolean isFinished() { 
        return finished;
    }

    }
