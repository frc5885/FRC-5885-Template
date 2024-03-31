package frc.robot.commands.test;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class TestArmDownCommand extends WCTestCommand {

  @Override
  boolean wasSuccessful() {
    boolean success = m_armSubsystem.isArmDown();
    if (!success) {
      DriverStation.reportError("Test: Arm did not report as being in the stow position!", false);
    }
    return success;
  }

  @Override
  long getDuration() {
    return 3000;
  }

  private ArmSubsystem m_armSubsystem;

  public TestArmDownCommand(
      ArmSubsystem armSubsystem,
      LEDSubsystem ledSubsystem
  ) {
    super(ledSubsystem);
    m_armSubsystem = armSubsystem;
  }

  @Override
  public void execute() {
    m_armSubsystem.pos(Constants.kArmStow);
  }

  @Override
  public boolean isFinished() {
    return super.isFinished() || m_armSubsystem.isArmDown();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_armSubsystem.stop();
  }
}
