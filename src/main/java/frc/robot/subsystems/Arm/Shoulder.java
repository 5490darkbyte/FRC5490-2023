package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shoulder extends SubsystemBase{

    private final TalonFX shoulderMotor = new TalonFX(RobotMap.shoulderMotor);

    
    
    private final CANCoder shoulderCANCoder = new CANCoder(RobotMap.shoulderEncoder);



    public Shoulder(){
        shoulderMotor.setInverted(true);
        shoulderMotor.setNeutralMode(NeutralMode.Brake);
        //What should timeout ms be?
       shoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0);

    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder CANCoder Absolute Position", shoulderCANCoder.getAbsolutePosition());
    }


    public void setShoulderMotor(double power){
        //shoulderMotor.set(ControlMode.PercentOutput, );
    }



    public double getShoulderPosition(){
        return shoulderCANCoder.getAbsolutePosition();
    }


    public boolean ShoulderisBack(){
        if (shoulderMotor.getSensorCollection().isRevLimitSwitchClosed() == 1)
        return true;
        else 
        return false;
    }

    //1587 for integrated encoder
    public boolean shoulderisStraight(){
        if (shoulderCANCoder.getAbsolutePosition() > 119)
        return true;
        else 
        return false;
    }


    
}
