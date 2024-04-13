package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.DriverController;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ShootCommand extends SequentialCommandGroup {

  // TODO turn off aimbot after shot

  public ShootCommand(
      Robot robot,
      DriverController driverController,
      ShooterSubsystem shooterSubsystem,
      WristSubsystem wristSubsystem,
      ArmSubsystem armSubsystem,
      PhotonVisionSystem photonVision,
      SwervePoseEstimator swervePoseEstimator,
      FeederSubsystem feederSubsystem,
      Beambreak beambreak) {
    addCommands(
        new AimShooterCommand(
            driverController,
            shooterSubsystem,
            robot,
            wristSubsystem,
            armSubsystem,
            photonVision,
            swervePoseEstimator,
            feederSubsystem,
            beambreak),
        new StowWristCommand(armSubsystem, wristSubsystem));
  }
}
