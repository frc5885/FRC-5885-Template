package frc.robot.commands.test;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class TestIntakeCommand extends WCTestCommand {

  public enum Type {
    INTAKE,
    OUTTAKE
  }

  @Override
  boolean wasSuccessful() {
    boolean success = Math.abs(m_intakeSubsystem.getVelocity()) > 100;
    if (!success) {
      DriverStation.reportError(
          "Test: Intake did not report as being above specified velocity threshold!", false);
    }
    return success;
  }

  @Override
  long getDuration() {
    return 3000;
  }

  private IntakeSubsystem m_intakeSubsystem;
  private Type type;

  public TestIntakeCommand(IntakeSubsystem intakeSubsystem, Type type, LEDSubsystem ledSubsystem) {
    super(ledSubsystem);
    m_intakeSubsystem = intakeSubsystem;
    this.type = type;
  }

  @Override
  public void execute() {
    if (type == Type.INTAKE) {
      m_intakeSubsystem.intake();
    } else {
      m_intakeSubsystem.outtake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_intakeSubsystem.stop();
  }
}
