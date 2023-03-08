// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //declare autonomous command variable
  private Command m_autonomousCommand;

  //declare robot container variable
  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {
    //Construct robot container object 
    m_robotContainer = new RobotContainer();

    //start camera feed
    CameraServer.startAutomaticCapture();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java SmartDashboard. 
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    //run auto command 
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  } 


  @Override
  public void autonomousPeriodic() {
    //Updates the field relative position of the robot 
    //based on the gyro position and swervemodule positions
    m_robotContainer.updateOdometry();
   
  }


   @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove this line or comment it out.
    if (m_autonomousCommand != null) 
      m_autonomousCommand.cancel();
  }
  
   @Override
  public void teleopPeriodic() {
    m_robotContainer.updateOdometry();
    m_robotContainer.driveWithJoystick(true);
  } 


  @Override
  public void disabledInit() {}


  @Override
  public void disabledPeriodic() {}

  
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}


  @Override
  public void simulationInit() {}


  @Override
  public void simulationPeriodic() {}
}
