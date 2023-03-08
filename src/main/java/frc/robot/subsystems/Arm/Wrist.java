package frc.robot.subsystems.Arm;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Wrist extends SubsystemBase{
        
    private final CANSparkMax wristMotor = new CANSparkMax(RobotMap.wristMotor, MotorType.kBrushless);
    

    private final CANCoder wristCANCoder = new CANCoder(RobotMap.wristEncoder);

    


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist CANCoder Absolute Position", wristCANCoder.getAbsolutePosition());


    }

    public Wrist(){
        wristMotor.setInverted(false);
        wristMotor.setIdleMode(IdleMode.kBrake);
    }
    

    public void setWristMotor(double power){
        wristMotor.set(power);
    }

       public boolean wristisUp(){
        if (wristCANCoder.getAbsolutePosition() < 261.73)
        return true;
        else 
        return false;
    }

    public double getWristPosition(){
        return wristCANCoder.getAbsolutePosition();
    }

    //wrist at same plane as forearm "straight" is 351.73
    public boolean wristisStraight(){
        if (wristCANCoder.getAbsolutePosition() < 354 && wristCANCoder.getAbsolutePosition() > 346)
        return true;
        else 
        return false;
    }

    
}
