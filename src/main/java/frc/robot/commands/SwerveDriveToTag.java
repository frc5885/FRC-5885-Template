// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import org.littletonrobotics.junction.Logger;

public class SwerveDriveToTag extends CommandBase {

  private final SwerveDrive m_swerveSubsystem;
  private final SwervePoseEstimator m_poseEstimator;

  private final SlewRateLimiter m_xAccelerationLimiter,
      m_yAccelerationLimiter,
      m_angularAccelerationLimiter;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveDriveToTag(SwerveDrive swerveSubsystem, SwervePoseEstimator poseEstimator) {
    m_swerveSubsystem = swerveSubsystem;
    m_poseEstimator = poseEstimator;

    m_xAccelerationLimiter =
        new SlewRateLimiter(SwerveConstants.kMaxAccelerationXMetersPerSecondSquared);
    m_yAccelerationLimiter =
        new SlewRateLimiter(SwerveConstants.kMaxAccelerationYMetersPerSecondSquared);
    m_angularAccelerationLimiter =
        new SlewRateLimiter(SwerveConstants.kMaxAccelerationAngularRadiansPerSecondSquared);

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpd =
        m_xAccelerationLimiter.calculate(
                MathUtil.clamp((14.513558 - m_poseEstimator.getPose().getX()) * 5, -1, 1))
            * 1;
    double ySpd =
        m_yAccelerationLimiter.calculate(
                MathUtil.clamp((4.424426 - m_poseEstimator.getPose().getY()) * 5, -1, 1))
            * 1;
    double turnSpd = 0;
    // double turnSpd =
    // m_angularAccelerationLimiter.calculate(MathUtil.clamp((0-m_poseEstimator.getPose().getRotation().getRadians())*0.05,-1, 1))
    //     * 3.14159;

    ChassisSpeeds chassisSpeeds;
    chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpd, ySpd, turnSpd, m_swerveSubsystem.getRotation2d());

    SwerveModuleState[] moduleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    Logger.getInstance().recordOutput("expected_module_states", moduleStates);
    m_swerveSubsystem.setModuleStates(moduleStates);
  }

  // 2024
  // https://github.wpilib.org/allwpilib/docs/development/java/edu/wpi/first/math/kinematics/ChassisSpeeds.html#discretize(edu.wpi.first.math.kinematics.ChassisSpeeds,double)
  // Right now
  // https://github.com/cachemoney8096/2023-charged-up/blob/main/src/main/java/frc/robot/subsystems/drive/DriveSubsystem.java#L174
  public static ChassisSpeeds discretize(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      double dtSeconds) {
    var desiredDeltaPose =
        new Pose2d(
            vxMetersPerSecond * dtSeconds,
            vyMetersPerSecond * dtSeconds,
            new Rotation2d(omegaRadiansPerSecond * dtSeconds));
    var twist = log(desiredDeltaPose);
    return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
  }

  /**
   * Logical inverse of the above. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/b5da3c760b78d598b492e1cc51d8331c2ad50f6a/src/main/java/com/team254/lib/geometry/Pose2d.java
   */
  public static Twist2d log(final Pose2d transform) {
    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < 1E-9) {
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halftheta_by_tan_of_halfdtheta =
          -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
    }
    final Translation2d translation_part =
        transform
            .getTranslation()
            .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta))
            .times(Math.hypot(halftheta_by_tan_of_halfdtheta, half_dtheta));

    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
