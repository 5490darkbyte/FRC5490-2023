package frc.robot.commands.ArmCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Wrist;


public class RaiseWristUp extends CommandBase {
    private final Wrist robotWrist; 

        
    public RaiseWristUp(Wrist robotWrist) {
        this.robotWrist = robotWrist;
        addRequirements(robotWrist);
    }


    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() { 
        robotWrist.setWristMotor(-.06);
        }
    

    @Override
    public void end(boolean interrupted) {
        robotWrist.setWristMotor(0);

    }

    @Override
    public boolean isFinished() {
        if(robotWrist.wristisUp())
        return true;
        else
        return false;
    }
 

}
