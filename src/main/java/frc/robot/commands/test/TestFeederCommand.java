package frc.robot.commands.test;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class TestFeederCommand extends WCTestCommand {

  public enum Type {
    INTAKE,
    OUTTAKE
  }

  @Override
  boolean wasSuccessful() {
    boolean success = Math.abs(m_feederSubsystem.getVelocity()) > 1000;
    if (!success) {
      DriverStation.reportError(
          "Test: Feeder did not report as being above specified velocity threshold!", false);
    }
    return success;
  }

  @Override
  long getDuration() {
    return 3000;
  }

  private FeederSubsystem m_feederSubsystem;
  private Type type;

  public TestFeederCommand(FeederSubsystem feederSubsystem, Type type, LEDSubsystem ledSubsystem) {
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
