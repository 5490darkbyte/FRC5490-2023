package frc.robot;

import frc.robot.commands.MakeRoboGo;
import frc.robot.commands.TurnWheelsForward;
import frc.robot.commands.ArmCommands.ArmUp;
import frc.robot.commands.ArmCommands.ElbowClearance;
import frc.robot.commands.ArmCommands.ElbowStraight;
import frc.robot.commands.ArmCommands.GripperClose;
import frc.robot.commands.ArmCommands.GripperOpen;
import frc.robot.commands.ArmCommands.ShoulderStraight;
import frc.robot.subsystems.Arm.Elbow;
import frc.robot.subsystems.Arm.Gripper;
import frc.robot.subsystems.Arm.Shoulder;
import frc.robot.subsystems.Arm.Wrist;
import frc.robot.subsystems.Drivetrain.Drivetrain2;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
    private final Drivetrain2 m_drivetrain = new Drivetrain2();
    private final Elbow robotElbow = new Elbow();
    private final Gripper robotGripper = new Gripper();
    private final Shoulder robotShoulder = new Shoulder();
    private final Wrist robotWrist = new Wrist();
    

    public XboxController driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);
	public XboxController armController = new XboxController(Constants.OIConstants.kArmControllerPort);

    JoystickButton XButton = new JoystickButton(armController, Constants.OIConstants.kArmXButton);
    JoystickButton YButton = new JoystickButton(armController, Constants.OIConstants.kArmYButton);


	
	public RobotContainer() {
      
         m_drivetrain.setDefaultCommand(new MakeRoboGo(m_drivetrain,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> driverController.getRightX()));

        configureButtonBindings();
      
    }


    private void configureButtonBindings() {   
       XButton.onTrue(new ArmUp(robotElbow, robotShoulder, robotWrist, robotGripper)); 
       YButton.onTrue(new ArmUp(robotElbow, robotShoulder, robotWrist, robotGripper));
        
    } 


    public Command getAutonomousCommand() {
        return new TurnWheelsForward(m_drivetrain);
        
        //return new SequentialCommandGroup(new GripperOpen(robotArm), new GripperClose(robotArm));
        
      }

}
