package frc.robot.commands.ArmCommands;

import frc.robot.subsystems.Arm.Elbow;
import frc.robot.subsystems.Arm.Gripper;
import frc.robot.subsystems.Arm.Shoulder;
import frc.robot.subsystems.Arm.Wrist;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class ArmUp extends SequentialCommandGroup {


  public ArmUp(Elbow robotElbow, Shoulder robotShoulder, Wrist robotWrist, Gripper robotGripper) {
    addCommands(new SequentialCommandGroup(
      new GripperClose(robotGripper), 
      new ElbowClearance(robotElbow),
      new ParallelCommandGroup(
        new ShoulderStraight(robotShoulder), 
        new ElbowStraight(robotElbow), 
        new WristStraight(robotWrist), 
        new GripperOpen(robotGripper))));


  }

    



}