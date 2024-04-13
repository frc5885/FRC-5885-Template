package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.OperatorController;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class OverrideShootCommand extends SequentialCommandGroup {

  // TODO turn off aimbot after shot

  public OverrideShootCommand(
      Robot robot,
      OperatorController operatorController,
      ShooterSubsystem shooterSubsystem,
      WristSubsystem wristSubsystem,
      ArmSubsystem armSubsystem,
      PhotonVisionSystem photonVision,
      SwervePoseEstimator swervePoseEstimator,
      Beambreak beambreak) {
    addCommands(
        new AimSpeakerShotCommand(
            operatorController,
            shooterSubsystem,
            robot,
            wristSubsystem,
            armSubsystem,
            photonVision,
            swervePoseEstimator,
            beambreak),
        new StowWristCommand(armSubsystem, wristSubsystem));
  }
}
