package frc.robot.subsystems.Arm;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Gripper extends SubsystemBase{

    private final CANSparkMax gripperMotor = new CANSparkMax(RobotMap.gripperMotor, MotorType.kBrushless);
    private final CANCoder gripperCANCoder = new CANCoder(RobotMap.gripperEncoder);
    


    public Gripper(){
        gripperMotor.setInverted(true);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gripper CANCoder Absolute Position", gripperCANCoder.getAbsolutePosition());

    }

        
    public void setGripperMotor(double power){
        gripperMotor.set(power);
    }

    public boolean gripperisOpen(){
        if (gripperCANCoder.getAbsolutePosition() < 233.52)
        return true;
        else 
        return false;
    }

    //fully closed at 274.48
    public boolean gripperisClosed(){
        if (gripperCANCoder.getAbsolutePosition() > 264.24)
        return true;
        else 
        return false;
    }
}
