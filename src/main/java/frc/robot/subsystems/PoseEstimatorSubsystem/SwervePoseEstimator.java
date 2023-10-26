// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PoseEstimatorSubsystem;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class SwervePoseEstimator extends SubsystemBase {
  private final Supplier<Rotation2d> m_rotationSupplier;
  private final Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final SwerveDrive m_sw;

  private final DoubleArraySubscriber m_observationSubscriber;

  /** Creates a new TankDrivePoseEstimator. */
  public SwervePoseEstimator(SwerveDrive swerveDrive) {

    m_rotationSupplier = swerveDrive::getRotation2d;
    m_swerveModulePositionSupplier = swerveDrive::getModulePositions;

    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveConstants.kDriveKinematics,
            m_rotationSupplier.get(),
            m_swerveModulePositionSupplier.get(),
            new Pose2d(),
            SwerveConstants.kStateStdDevs,
            SwerveConstants.kVisionMeasurementStdDevs);

    m_sw = swerveDrive;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("NoodleVision/output");
    m_observationSubscriber = table.getDoubleArrayTopic("observations").subscribe(new double[] {});
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(m_rotationSupplier.get(), m_swerveModulePositionSupplier.get());

    TimestampedDoubleArray obs_tm = m_observationSubscriber.getAtomic();
    // System.out.println("obs_tm: " + obs_tm.value.length + " " + obs_tm.timestamp );
    if (obs_tm.value.length > 1) {
      double timestmp = obs_tm.timestamp / 1000000.0;
      Quaternion q =
          new Quaternion(obs_tm.value[7], obs_tm.value[4], obs_tm.value[5], obs_tm.value[6]);
      Pose2d pos1 =
          new Pose2d(obs_tm.value[1], obs_tm.value[2], new Rotation2d(new Rotation3d(q).getZ()));
      Quaternion q2 =
          new Quaternion(obs_tm.value[14], obs_tm.value[11], obs_tm.value[12], obs_tm.value[13]);
      Pose2d pos2 =
          new Pose2d(obs_tm.value[8], obs_tm.value[9], new Rotation2d(new Rotation3d(q2).getZ()));

      Pose2d using = m_poseEstimator.getEstimatedPosition().nearest(List.of(pos1, pos2));
      Pose2d camera_loc = new Pose2d(SwerveConstants.kTrackWidthMeters / 2, 0, new Rotation2d());
      using =
          using.transformBy(
              (new Transform2d(camera_loc.getTranslation(), camera_loc.getRotation()).inverse()));

      m_poseEstimator.addVisionMeasurement(using, timestmp);
    }

    Logger.getInstance().recordOutput("Pose Estimator", m_poseEstimator.getEstimatedPosition());
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void reset(Pose2d newPos) {
    m_sw.resetGyro();
    m_poseEstimator.resetPosition(
        m_rotationSupplier.get(), m_swerveModulePositionSupplier.get(), newPos);
  }
}
