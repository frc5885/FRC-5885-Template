package frc.robot.commands.test;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class TestWristAimCommand extends WCTestCommand {

  @Override
  boolean wasSuccessful() {
    boolean success = m_wristSubsystem.isAtPos();
    if (!success) {
      DriverStation.reportError("Test: Wrist never reached specified position!", false);
    }
    return success;
  }

  @Override
  long getDuration() {
    return 3000;
  }

  private final WristSubsystem m_wristSubsystem;

  public TestWristAimCommand(WristSubsystem wristSubsystem, LEDSubsystem ledSubsystem) {
    super(ledSubsystem);
    m_wristSubsystem = wristSubsystem;
  }

  @Override
  public void execute() {
    m_wristSubsystem.pos(0.34);
  }

  @Override
  public boolean isFinished() {
    return super.isFinished() || m_wristSubsystem.isAtPos();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_wristSubsystem.stop();
  }
}
