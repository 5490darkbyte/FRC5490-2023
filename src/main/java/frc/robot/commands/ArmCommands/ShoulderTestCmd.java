package frc.robot.commands.ArmCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Shoulder;

public class ShoulderTestCmd extends CommandBase {
    /**
     *
     */
    private final Shoulder robotShoulder; 


    public ShoulderTestCmd(Shoulder robotShoulder) {
        this.robotShoulder = robotShoulder;
        addRequirements(robotShoulder);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() { 
        robotShoulder.turnShoulderToDegrees(119);
        }
    

    @Override
    public void end(boolean interrupted) {
        

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
    

