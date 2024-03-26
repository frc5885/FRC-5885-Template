package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.DriverController;
import frc.robot.base.io.OperatorController;
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
      Beambreak beambreak) {
    addCommands(
        new AimSpeakerShotCommand(
            operatorController,
            shooterSubsystem,
            robot,
            wristSubsystem,
            armSubsystem,
            beambreak),
        new StowWristCommand(armSubsystem, wristSubsystem));
  }
}
