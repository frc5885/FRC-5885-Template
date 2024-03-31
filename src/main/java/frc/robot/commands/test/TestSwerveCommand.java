package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.modules.swerve.SwerveConstants;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class TestSwerveCommand extends WCTestCommand {

  public enum Type {
    FORWARD,
    BACKWARD,
    ROTATE_CLOCKWISE,
    ROTATE_COUNTER_CLOCKWISE
  }

  private SwerveDriveSubsystem m_swerveDriveSubsystem;
  private final Type m_type;

  @Override
  boolean wasSuccessful() {
    SwerveModulePosition[] positions = m_swerveDriveSubsystem.getModulePositions();
    double[] values = new double[4];
    for (int i = 0; i <= 3; i++) {
      double value;
      if (m_type == Type.FORWARD || m_type == Type.BACKWARD) {
        value = Math.abs(positions[i].distanceMeters);
      } else {
        value = Math.abs(positions[i].angle.getDegrees()) % 360;
      }
      values[i] = value;
    }
    double total = values[0] + values[1] + values[2] + values[3];
    double totalDiv = total / 4;

    double buffer;
    if (m_type == Type.FORWARD || m_type == Type.BACKWARD) {
      buffer = 0.5; // Meters
    } else {
      buffer = 3.0; // Degrees
    }

    for (int i = 0; i <= 3; i++) {
      if (values[i] < totalDiv - buffer || values[i] > totalDiv + buffer) {
        DriverStation.reportError("Test: 1 or more swerve modules was out of sync from the others!", false);
        return false;
      }
    }

    return true;
  }

  @Override
  long getDuration() {
    return 3000;
  }

  public TestSwerveCommand(
      SwerveDriveSubsystem swerveDriveSubsystem,
      Type type,
      LEDSubsystem ledSubsystem
  ) {
    super(ledSubsystem);
    m_swerveDriveSubsystem = swerveDriveSubsystem;
    m_type = type;
  }

  @Override
  public void execute() {
    double xVel = 0;

    switch (m_type) {
      case FORWARD -> xVel = SwerveConstants.kMaxSpeedMetersPerSecond;
      case BACKWARD -> xVel = -SwerveConstants.kMaxSpeedMetersPerSecond;
      case ROTATE_CLOCKWISE -> {
        m_swerveDriveSubsystem.setTurnVoltage(6);
        return;
      }
      case ROTATE_COUNTER_CLOCKWISE -> {
        m_swerveDriveSubsystem.setTurnVoltage(-6);
        return;
      }
    }
    m_swerveDriveSubsystem.setModuleStates(
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromRobotRelativeSpeeds(
                xVel,
                0,
                0,
                Rotation2d.fromDegrees(0)
            )
        )
    );
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_swerveDriveSubsystem.setModuleStates(
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromRobotRelativeSpeeds(
                0,
                0,
                0,
                Rotation2d.fromDegrees(0)
            )
        )
    );
  }
}
