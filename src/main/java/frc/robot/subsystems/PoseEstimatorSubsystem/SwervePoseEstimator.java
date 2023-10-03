// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PoseEstimatorSubsystem;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class SwervePoseEstimator extends SubsystemBase {
  private final Supplier<Rotation2d> m_rotationSupplier;
  private final Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  /** Creates a new TankDrivePoseEstimator. */
  public SwervePoseEstimator(SwerveDrive swerveDrive) {

    m_rotationSupplier = () -> (swerveDrive.getRotation2d());
    m_swerveModulePositionSupplier = () -> (swerveDrive.getModulePositions());

    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveConstants.kDriveKinematics,
            m_rotationSupplier.get(),
            m_swerveModulePositionSupplier.get(),
            new Pose2d(),
            SwerveConstants.kStateStdDevs,
            SwerveConstants.kVisionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(m_rotationSupplier.get(), m_swerveModulePositionSupplier.get());

    Logger.getInstance().recordOutput("Pose Estimator", m_poseEstimator.getEstimatedPosition());
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void reset(Pose2d newPos) {
    m_poseEstimator.resetPosition(
        m_rotationSupplier.get(), m_swerveModulePositionSupplier.get(), newPos);
  }
}
