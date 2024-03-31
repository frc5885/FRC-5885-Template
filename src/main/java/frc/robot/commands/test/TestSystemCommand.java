package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.commands.DwellCommand;
import frc.robot.subsystems.*;

public class TestSystemCommand extends SequentialCommandGroup {
  private static final long dwell = 1500;
  public TestSystemCommand(
      SwerveDriveSubsystem swerveDriveSubsystem,
      ArmSubsystem armSubsystem,
      WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      FeederSubsystem feederSubsystem,
      LEDSubsystem ledSubsystem
  ) {
    super(
        // Swerve
        new TestResetSwerveCommand(swerveDriveSubsystem),
        new TestSwerveCommand(swerveDriveSubsystem, TestSwerveCommand.Type.FORWARD, ledSubsystem),
        new DwellCommand(dwell),
        new TestResetSwerveCommand(swerveDriveSubsystem),
        new TestSwerveCommand(swerveDriveSubsystem, TestSwerveCommand.Type.BACKWARD, ledSubsystem),
        new DwellCommand(dwell),
        new TestResetSwerveCommand(swerveDriveSubsystem),
        new TestSwerveCommand(swerveDriveSubsystem, TestSwerveCommand.Type.ROTATE_CLOCKWISE, ledSubsystem),
        new DwellCommand(dwell),
        new TestResetSwerveCommand(swerveDriveSubsystem),
        new TestSwerveCommand(swerveDriveSubsystem, TestSwerveCommand.Type.ROTATE_COUNTER_CLOCKWISE, ledSubsystem),

        // Arm
        new DwellCommand(dwell),
        new TestArmUpCommand(armSubsystem, ledSubsystem),
        new DwellCommand(dwell),
        new TestArmDownCommand(armSubsystem, ledSubsystem),

        // Wrist
        new DwellCommand(dwell),
        new TestWristAimCommand(wristSubsystem, ledSubsystem),
        new DwellCommand(dwell),
        new TestWristStowCommand(wristSubsystem, ledSubsystem),

        // Shooter
        new DwellCommand(dwell),
        new TestShooterCommand(shooterSubsystem, ledSubsystem),

        // Intake
        new DwellCommand(dwell),
        new TestIntakeCommand(intakeSubsystem, TestIntakeCommand.Type.INTAKE, ledSubsystem),
        new DwellCommand(dwell),
        new TestIntakeCommand(intakeSubsystem, TestIntakeCommand.Type.OUTTAKE, ledSubsystem),

        // Feeder
        new DwellCommand(dwell),
        new TestFeederCommand(feederSubsystem, TestFeederCommand.Type.INTAKE, ledSubsystem),
        new DwellCommand(dwell),
        new TestFeederCommand(feederSubsystem, TestFeederCommand.Type.OUTTAKE, ledSubsystem),

        // End
        new DwellCommand(dwell),
        new InstantCommand(
            ledSubsystem::setRainbow
        )
    );
  }
}
