// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.debug.PoseEstimatorSubsystem;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoConstants.PoseEstimatorConstants;
import frc.robot.base.modules.swerve.SwerveConstants;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;

/** Add your docs here. */
public class SwervePoseEstimator extends SubsystemBase {
  private final Supplier<Rotation2d> m_rotationSupplier;
  private final Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final SwerveDriveSubsystem m_swerveDrive;

  private final Vision m_photonVision;

  /** Creates a new TankDrivePoseEstimator. */
  public SwervePoseEstimator(SwerveDriveSubsystem swerveDrive) {

    m_rotationSupplier = swerveDrive::getRotation2d;
    m_swerveModulePositionSupplier = swerveDrive::getModulePositions;

    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveConstants.kDriveKinematics,
            m_rotationSupplier.get(),
            m_swerveModulePositionSupplier.get(),
            new Pose2d(),
            PoseEstimatorConstants.kEncoderMeasurementStdDevs,
            PoseEstimatorConstants.kVisionMeasurementStdDevs);

    m_swerveDrive = swerveDrive;

    m_photonVision = new Vision();

    reset();
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(m_rotationSupplier.get(), m_swerveModulePositionSupplier.get());

    if (m_photonVision.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition()).isPresent()) {
      EstimatedRobotPose estimatedRobotPose =
          m_photonVision.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition()).get();
      m_poseEstimator.addVisionMeasurement(
          estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
    }
  }

  public void addVisionPose(Pose2d pose, double timestamp) {
    m_poseEstimator.addVisionMeasurement(pose, timestamp);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void reset(Pose2d newPos) {
    m_swerveDrive.resetGyro();
    m_poseEstimator.resetPosition(
        m_rotationSupplier.get(), m_swerveModulePositionSupplier.get(), newPos);
  }

  public void reset() {
    reset(new Pose2d(0, 0, new Rotation2d()));
  }
}
