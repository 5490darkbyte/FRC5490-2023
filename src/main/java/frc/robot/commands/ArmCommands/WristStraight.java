package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Wrist;


public class WristStraight extends CommandBase {
    private final Wrist robotWrist; 

        
    public WristStraight(Wrist robotWrist) {
        this.robotWrist = robotWrist;
        addRequirements(robotWrist);
    }


    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() { 
        if (robotWrist.getWristPosition() > 354 || robotWrist.getWristPosition() < 77)
        robotWrist.setWristMotor(-.06);
        else if ( robotWrist.getWristPosition() < 346)
        robotWrist.setWristMotor(.06);
        }
    

    @Override
    public void end(boolean interrupted) {
        robotWrist.setWristMotor(0);

    }

    @Override
    public boolean isFinished() {
        if(robotWrist.wristisStraight())
        return true;
        else
        return false;
    }

    //wrist at same plane as forearm "straight" is 351.73

}