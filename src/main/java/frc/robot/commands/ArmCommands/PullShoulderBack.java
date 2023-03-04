package frc.robot.commands.ArmCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Shoulder;

public class PullShoulderBack extends CommandBase {
    private final Shoulder robotShoulder; 


    public PullShoulderBack(Shoulder robotShoulder) {
        this.robotShoulder = robotShoulder;
        addRequirements(robotShoulder);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() { 
        robotShoulder.setShoulderMotor(.06);
        }
    

    @Override
    public void end(boolean interrupted) {
        robotShoulder.setShoulderMotor(0);

    }

    @Override
    public boolean isFinished() {
        if(robotShoulder.ShoulderisBack())
        return true;
        else
        return false;
    }

}
    

