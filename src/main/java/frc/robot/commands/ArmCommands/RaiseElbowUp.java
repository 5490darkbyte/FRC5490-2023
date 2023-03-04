package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Elbow;

public class RaiseElbowUp extends CommandBase {
    private final Elbow robotElbow; 


    public RaiseElbowUp(Elbow robotElbow) {
        this.robotElbow = robotElbow;
        addRequirements(robotElbow);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() { 
        robotElbow.setElbowMotor(-.06);
        }
    

    @Override
    public void end(boolean interrupted) {
        robotElbow.setElbowMotor(0);

    }

    @Override
    public boolean isFinished() {
        if(robotElbow.elbowisUp())
        return true;
        else
        return false;
    }
 
    
}
