package frc.robot;

import frc.robot.commands.DriveForward;
import frc.robot.commands.MakeRoboGo;
import frc.robot.commands.TurnWheelsForward;
import frc.robot.commands.ArmCommands.ArmUp;
import frc.robot.subsystems.Arm.Elbow;
import frc.robot.subsystems.Arm.Gripper;
import frc.robot.subsystems.Arm.Shoulder;
import frc.robot.subsystems.Arm.Wrist;
import frc.robot.subsystems.Drivetrain.Drivetrain2;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotContainer {
    //Initialize subsystem objects
    private final Drivetrain2 m_drivetrain = new Drivetrain2();
    private final Elbow robotElbow = new Elbow();
    private final Gripper robotGripper = new Gripper();
    private final Shoulder robotShoulder = new Shoulder();
    private final Wrist robotWrist = new Wrist();
    
    //Initialize XboxController objects
    public GenericHID driverController = new GenericHID(Constants.OIConstants.kDriverControllerPort);
	public XboxController armController = new XboxController(Constants.OIConstants.kArmControllerPort);

    //Initialize JoystickButton objects
    JoystickButton XButton = new JoystickButton(armController, Constants.OIConstants.kArmXButton);
    JoystickButton YButton = new JoystickButton(armController, Constants.OIConstants.kArmYButton);

    //Initialize Sendanle Chooser to select auto run from SmartDashboard
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    //Design auto run options
    private final Command m_simpleTaxi_Auto = new SequentialCommandGroup(new TurnWheelsForward(m_drivetrain), new DriveForward(m_drivetrain));
    private final Command m_noMove_Auto = new TurnWheelsForward(m_drivetrain); 


	public RobotContainer() {
        //Create auto option buttons on SmartDashboard
        m_chooser.setDefaultOption("Simple Taxi", m_simpleTaxi_Auto);
        m_chooser.addOption("No Move", m_noMove_Auto);
        SmartDashboard.putData("Auto Choices", m_chooser);

        configureButtonBindings();
      
    }

    //button commands for arm control
    private void configureButtonBindings() {   
      XButton.onTrue(new ArmUp(robotElbow, robotShoulder, robotWrist, robotGripper)); 
      YButton.onTrue(new ArmUp(robotElbow, robotShoulder, robotWrist, robotGripper));     
    } 


    //returns the commands to be run during auto
    public Command getAutonomousCommand() {
       return m_chooser.getSelected();

    }

    public void updateOdometry(){
        //Updates the field relative position of the robot 
        //based on the gyro position and swervemodule positions
        m_drivetrain.updateOdometry();
    }

    //get values from xbox controller and call MakeRoboGo command with them
    public void driveWithJoystick(boolean fieldRelative) {
        final var xSpeed = driverController.getRawAxis(1);
        final var ySpeed = driverController.getRawAxis(0);
        final var rot = driverController.getRawAxis(4);

        SmartDashboard.putNumber("xSpeed", driverController.getRawAxis(1));
        SmartDashboard.putNumber("ySpeed", driverController.getRawAxis(0));
        SmartDashboard.putNumber("rot", driverController.getRawAxis(4));

        new MakeRoboGo(m_drivetrain, xSpeed, ySpeed, rot, fieldRelative).schedule();
}
  }
