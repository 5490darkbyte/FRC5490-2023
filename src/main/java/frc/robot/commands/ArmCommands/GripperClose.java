package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Gripper;


public class GripperClose extends CommandBase {
    private final Gripper robotGripper;
    

        
    public GripperClose(Gripper  robotGripper) {
        this.robotGripper =  robotGripper;
        addRequirements( robotGripper);
    }


    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() { 
        robotGripper.setGripperMotor(-.05);
    }


    

    @Override
    public void end(boolean interrupted) {
        robotGripper.setGripperMotor(0);

    }

    @Override
    public boolean isFinished() {
        if(robotGripper.gripperisClosed())
            return true;
        else
        return false;
    }

}
