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
        robotShoulder.turnShoulderToDegrees(0);
        }
    

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
    

