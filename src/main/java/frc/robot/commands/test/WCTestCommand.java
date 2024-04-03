package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public abstract class WCTestCommand extends Command {
  protected long startTime = 0;

  abstract long getDuration();

  abstract boolean wasSuccessful();

  private LEDSubsystem m_ledSubsystem;

  public WCTestCommand(LEDSubsystem ledSubsystem) {
    m_ledSubsystem = ledSubsystem;
  }

  @Override
  public final void initialize() {
    startTime = System.currentTimeMillis();
    m_ledSubsystem.setMode(LEDSubsystem.LEDMode.TESTING);
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime >= getDuration();
  }

  @Override
  public void end(boolean interrupted) {
    if (wasSuccessful()) {
      m_ledSubsystem.setMode(LEDSubsystem.LEDMode.SUCCESS);
    } else {
      m_ledSubsystem.setMode(LEDSubsystem.LEDMode.ERROR);
    }
  }
}
