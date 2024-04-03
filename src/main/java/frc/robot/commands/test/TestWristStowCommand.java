package frc.robot.commands.test;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class TestWristStowCommand extends WCTestCommand {

  @Override
  boolean wasSuccessful() {
    boolean success = m_wristSubsystem.isStowed();
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

  public TestWristStowCommand(WristSubsystem wristSubsystem, LEDSubsystem ledSubsystem) {
    super(ledSubsystem);
    m_wristSubsystem = wristSubsystem;
  }

  @Override
  public void execute() {
    m_wristSubsystem.pos(Constants.kWristStow);
  }

  @Override
  public boolean isFinished() {
    return super.isFinished() || m_wristSubsystem.isStowed();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_wristSubsystem.stop();
  }
}
