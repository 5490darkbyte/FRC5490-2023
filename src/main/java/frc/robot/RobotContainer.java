package frc.robot;

import frc.robot.commands.MakeRoboGo;
import frc.robot.subsystems.Drivetrain.Drivetrain2;
import edu.wpi.first.wpilibj.XboxController;


/* import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.Supplier; */


public class RobotContainer {
    private final Drivetrain2 m_drivetrain = new Drivetrain2();
    

    public XboxController driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);
	public XboxController armController = new XboxController(Constants.OIConstants.kArmControllerPort);


	
	public RobotContainer() {
        m_drivetrain.setDefaultCommand(new MakeRoboGo(m_drivetrain,
            () -> -driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> driverController.getRightX()));

        configureButtonBindings();
    }

    private void configureButtonBindings() {} 

}
