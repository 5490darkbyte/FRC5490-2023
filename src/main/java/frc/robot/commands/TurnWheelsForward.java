
package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.Drivetrain2;


public class TurnWheelsForward extends CommandBase {
    private final Drivetrain2 SwerveDrive;



    public TurnWheelsForward(Drivetrain2 SwerveDrive) {
        this.SwerveDrive = SwerveDrive;
        addRequirements(SwerveDrive);
    }

    @Override
    public void initialize() {
        SwerveDrive.resetEncodersToAbsolutes();
        
        //put motors in brake mode
        SwerveDrive.enableMotors(true);

        //Stop drive motors
        SwerveDrive.setDriveMotors(0);

        SwerveDrive.resetRelativeEncoders();


        
    }

    @Override
    public void execute() {
    SmartDashboard.putString("Flag", "working");
       //Turn all wheels to straight forward
       SwerveDrive.setTurnMotors(.1);


    }


   
    @Override
    public void end(boolean interrupted) {
        SwerveDrive.resetRelativeEncoders();
    }


    @Override
    public boolean isFinished() { 
        return SwerveDrive.WheelsAreFoward();
    


}
    public Drivetrain2 getDrivetrain(){
        return this.SwerveDrive;
    }
}

