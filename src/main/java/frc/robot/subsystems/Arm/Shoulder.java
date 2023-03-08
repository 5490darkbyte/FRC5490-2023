package frc.robot.subsystems.Arm;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.mathFunctions;
import frc.robot.Constants.DriveConstants;



public class Shoulder extends SubsystemBase{

    private final TalonFX shoulderMotor = new TalonFX(RobotMap.shoulderMotor);

    public final PIDController shoulderPidController;
    
    private final CANCoder shoulderCANCoder = new CANCoder(RobotMap.shoulderEncoder);



    public Shoulder(){
        shoulderMotor.setInverted(true);
        shoulderMotor.setNeutralMode(NeutralMode.Brake);
        //What should timeout ms be?
       shoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0);
              
       //instantiate PID for turning motor
        shoulderPidController = new PIDController(DriveConstants.kShoulderPIDProportion, 0, 0);
        shoulderPidController.enableContinuousInput(-180,180);
        shoulderPidController.setTolerance(1);

    }

    public void turnShoulderToDegrees(double desiredDegree){
        shoulderPidController.setSetpoint(desiredDegree);
        setShoulderMotor();
    }
  


    @Override
    public void periodic() {

        SmartDashboard.putNumber("Shoulder CANCoder Absolute Position Radians", shoulderCANCoder.getAbsolutePosition());
        SmartDashboard.putNumber("Shoulder CANCoder Absolute Position Degrees", mathFunctions.turningPosRadToDegrees(shoulderCANCoder.getAbsolutePosition()));
    }


    public void setShoulderMotor(){
        shoulderMotor.set(ControlMode.PercentOutput, shoulderPidController.calculate(mathFunctions.turningPosRadToDegrees(shoulderCANCoder.getAbsolutePosition())));
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


    public boolean shoulderisStraight(){
        if (shoulderCANCoder.getAbsolutePosition() > 119)
        return true;
        else 
        return false;
    }


    
}
