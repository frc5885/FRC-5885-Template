package frc.robot.commands.test;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;

public class TestResetSwerveCommand extends Command {

  private SwerveDriveSubsystem m_swerveDriveSubsystem;
  public TestResetSwerveCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
    m_swerveDriveSubsystem = swerveDriveSubsystem;
  }

  private long startTime = 0;
  private long delay = 1000;

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    m_swerveDriveSubsystem.setModulesAngle(0);
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime > delay;
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDriveSubsystem.stop();
  }
}
