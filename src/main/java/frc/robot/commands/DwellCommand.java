package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class DwellCommand extends Command {
  private long start;
  private long duration;

  public DwellCommand(long duration) {
    this.duration = duration;
  }

  @Override
  public void initialize() {
    start = System.currentTimeMillis();
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - start >= duration;
  }
}
