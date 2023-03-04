package frc.robot.commands.ArmCommands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Shoulder;

public class ShoulderStraight extends CommandBase {
    private final Shoulder robotShoulder; 


    public ShoulderStraight(Shoulder robotShoulder) {
        this.robotShoulder = robotShoulder;
        addRequirements(robotShoulder);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() { 
        robotShoulder.setShoulderMotor(0);
        
        //robotShoulder.setShoulderMotor(.17);
        }
    

    @Override
    public void end(boolean interrupted) {
        
        robotShoulder.setShoulderMotor(0);

    }

    @Override
    public boolean isFinished() {
        //if(robotShoulder.shoulderisStraight())
       // return true;
       // else
        return false;
    }

     //Position measured when arm is horizontal 1587

}