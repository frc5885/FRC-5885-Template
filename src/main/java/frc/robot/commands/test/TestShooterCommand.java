package frc.robot.commands.test;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TestShooterCommand extends WCTestCommand {

  @Override
  boolean wasSuccessful() {
    boolean success = m_shooterSubsystem.isVelocityTerminal();
    if (!success) {
      DriverStation.reportError("Test: Shooter(s) did not reach terminal velocity!", false);
    }
    return success;
  }

  @Override
  long getDuration() {
    return 4000;
  }

  private ShooterSubsystem m_shooterSubsystem;

  public TestShooterCommand(
      ShooterSubsystem shooterSubsystem,
      LEDSubsystem ledSubsystem
  ) {
    super(ledSubsystem);
    m_shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void execute() {
    m_shooterSubsystem.spinFastClose();
  }

  @Override
  public boolean isFinished() {
    return super.isFinished() || m_shooterSubsystem.isVelocityTerminal();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_shooterSubsystem.stop();
  }
}
