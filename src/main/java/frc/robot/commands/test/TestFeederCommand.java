package frc.robot.commands.test;

import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class TestFeederCommand extends WCTestCommand {

  public enum Type {
    INTAKE,
    OUTTAKE
  }

  @Override
  boolean wasSuccessful() {
    return true;
  }

  @Override
  long getDuration() {
    return 3000;
  }

  private FeederSubsystem m_feederSubsystem;
  private Type type;

  public TestFeederCommand(
      FeederSubsystem feederSubsystem,
      Type type,
      LEDSubsystem ledSubsystem
  ) {
    super(ledSubsystem);
    m_feederSubsystem = feederSubsystem;
    this.type = type;
  }

  @Override
  public void execute() {
    if (type == Type.INTAKE) {
      m_feederSubsystem.intake();
    } else {
      m_feederSubsystem.outtake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_feederSubsystem.stop();
  }
}
