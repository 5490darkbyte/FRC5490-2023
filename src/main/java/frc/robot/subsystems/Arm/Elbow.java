package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elbow extends SubsystemBase {
    private final TalonFX elbowMotor = new TalonFX(RobotMap.elbowMotor);
    private final CANCoder elbowCANCoder = new CANCoder(RobotMap.elbowEncoder);


@Override
public void periodic() {
    SmartDashboard.putNumber("Elbow CANCoder Absolute Position", elbowCANCoder.getAbsolutePosition());

    
}

public Elbow(){
    elbowMotor.setInverted(false);
    elbowMotor.setNeutralMode(NeutralMode.Brake);
}



public void setElbowMotor(double power){
    elbowMotor.set(ControlMode.PercentOutput, power);
}


public boolean elbowisUp(){
    if (elbowCANCoder.getAbsolutePosition() > 187.2)
    return true;
    else 
    return false;
}


public double getElbowPosition(){
    return elbowCANCoder.getAbsolutePosition();
}


//goal is 217.2
public boolean elbowisClear(){
    if (elbowCANCoder.getAbsolutePosition() > 216 && elbowCANCoder.getAbsolutePosition() < 219)
    return true;
    else 
    return false;
}


public boolean elbowisStraight(){
    if (elbowCANCoder.getAbsolutePosition() < 131.2)
    return true;
    else 
    return false;
}

}

