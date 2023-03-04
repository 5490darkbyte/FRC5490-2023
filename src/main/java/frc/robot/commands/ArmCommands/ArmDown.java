package frc.robot.commands.ArmCommands;
import frc.robot.subsystems.Arm.Elbow;
import frc.robot.subsystems.Arm.Gripper;
import frc.robot.subsystems.Arm.Shoulder;
import frc.robot.subsystems.Arm.Wrist;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class ArmDown extends SequentialCommandGroup {


  public ArmDown(Elbow robotElbow, Shoulder robotShoulder, Wrist robotWrist, Gripper robotGripper) {
    addCommands(new SequentialCommandGroup( 
      new ParallelCommandGroup(
          new GripperClose(robotGripper),
          new ElbowClearance(robotElbow), 
          new PullShoulderBack(robotShoulder)),
          new RaiseWristUp(robotWrist)),
      new RaiseElbowUp(robotElbow));

  }

}
