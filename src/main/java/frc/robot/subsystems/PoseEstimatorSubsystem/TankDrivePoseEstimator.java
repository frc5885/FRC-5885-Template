// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PoseEstimatorSubsystem;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TankConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TankDrivePoseEstimator extends SubsystemBase {

  private final Supplier<Rotation2d> m_rotationSupplier;
  private final Supplier<Double> m_leftDistanceSupplier;
  private final Supplier<Double> m_rightDistanceSupplier;

  private final DifferentialDrivePoseEstimator m_poseEstimator;

  /** Creates a new TankDrivePoseEstimator. */
  public TankDrivePoseEstimator(
      Supplier<Rotation2d> rotationSupplier,
      Supplier<Double> leftDistanceSupplier,
      Supplier<Double> rightDistanceSupplier) {

    m_rotationSupplier = rotationSupplier;
    m_leftDistanceSupplier = leftDistanceSupplier;
    m_rightDistanceSupplier = rightDistanceSupplier;

    m_poseEstimator =
        new DifferentialDrivePoseEstimator(
            TankConstants.Auto.kDriveKinematics,
            m_rotationSupplier.get(),
            m_leftDistanceSupplier.get(),
            m_rightDistanceSupplier.get(),
            new Pose2d(),
            TankConstants.Auto.stateStdDevs,
            TankConstants.Auto.visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(
        m_rotationSupplier.get(), m_leftDistanceSupplier.get(), m_rightDistanceSupplier.get());

    Logger.getInstance().recordOutput("Pose Estimator", m_poseEstimator.getEstimatedPosition());
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void setPos(Pose2d newPos) {
    m_poseEstimator.resetPosition(
        m_rotationSupplier.get(),
        m_leftDistanceSupplier.get(),
        m_rightDistanceSupplier.get(),
        newPos);
  }

  public void resetPos() {
    m_poseEstimator.resetPosition(new Rotation2d(), 0, 0, new Pose2d());
  }
}
