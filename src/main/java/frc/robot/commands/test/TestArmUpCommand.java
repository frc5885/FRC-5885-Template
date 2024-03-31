package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.base.modules.swerve.SwerveConstants;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class TestArmUpCommand extends WCTestCommand {

  @Override
  boolean wasSuccessful() {
    boolean success = m_armSubsystem.isArmUp();
    if (!success) {
      DriverStation.reportError("Test: Arm did not report as being in the amp position!", false);
    }
    return success;
  }

  @Override
  long getDuration() {
    return 3000;
  }

  private ArmSubsystem m_armSubsystem;

  public TestArmUpCommand(
      ArmSubsystem armSubsystem,
      LEDSubsystem ledSubsystem
  ) {
    super(ledSubsystem);
    m_armSubsystem = armSubsystem;
  }

  @Override
  public void execute() {
    m_armSubsystem.pos(Constants.kArmAmp);
  }

  @Override
  public boolean isFinished() {
    return super.isFinished() || m_armSubsystem.isArmUp();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_armSubsystem.stop();
  }
}
