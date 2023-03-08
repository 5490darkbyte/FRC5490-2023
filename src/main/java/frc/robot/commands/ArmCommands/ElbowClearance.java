package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Elbow;

public class ElbowClearance extends CommandBase {
    private final Elbow robotElbow; 


    public ElbowClearance(Elbow robotElbow) {
        this.robotElbow = robotElbow;
        addRequirements(robotElbow);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() { 
        if (robotElbow.getElbowPosition() < 213)
        robotElbow.setElbowMotor(-.06);
        else if (robotElbow.getElbowPosition() > 217)
        robotElbow.setElbowMotor(.06);
     
    }
    

    @Override
    public void end(boolean interrupted) {
        robotElbow.setElbowMotor(0);

    }

    @Override
    public boolean isFinished() {
       if(robotElbow.elbowisClear())
         return true;
        else
        return false;
    }
 
    
}
