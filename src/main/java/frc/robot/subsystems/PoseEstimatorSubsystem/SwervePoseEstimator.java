// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PoseEstimatorSubsystem;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PoseEstimatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class SwervePoseEstimator extends SubsystemBase {
  private final Supplier<Rotation2d> m_rotationSupplier;
  private final Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final SwerveDrive m_swerveDrive;

  private final DoubleArraySubscriber m_observationSubscriber;

  // private final IntegerArraySubscriber m_visibleTagsSubscriber;

  // private final AprilTagFieldLayout m_aprilTagFieldLayout;

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
            PoseEstimatorConstants.kEncoderMeasurementStdDevs,
            PoseEstimatorConstants.kVisionMeasurementStdDevs);

    m_swerveDrive = swerveDrive;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("NoodleVision1/output");
    m_observationSubscriber =
        table.getDoubleArrayTopic("SwervePoseEstimator/observations").subscribe(new double[] {});
    // m_visibleTagsSubscriber =
    //     table.getIntegerArrayTopic("SwervePoseEstimator/visibleTags").subscribe(new long[] {});
    // m_aprilTagFieldLayout = AprilTagFields.
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "SwervePoseEstimator/estimatedPose", m_poseEstimator.getEstimatedPosition());
    m_poseEstimator.update(m_rotationSupplier.get(), m_swerveModulePositionSupplier.get());
    // System.out.println(m_visibleTagsSubscriber.get().length);

    TimestampedDoubleArray obs_tm = m_observationSubscriber.getAtomic();
    // System.out.println("obs_tm: " + obs_tm.value.length + " " + obs_tm.timestamp);
    if (obs_tm.value.length >= 1) {
      double timestmp = obs_tm.timestamp / 1000000.0;

      Pose3d cameraPose = null;

      if (obs_tm.value[0] == 1) {
        cameraPose =
            new Pose3d(
                obs_tm.value[2],
                obs_tm.value[3],
                obs_tm.value[4],
                new Rotation3d(
                    new Quaternion(
                        obs_tm.value[5], obs_tm.value[6], obs_tm.value[7], obs_tm.value[8])));
      }

      if (obs_tm.value[0] == 2) {
        double error0 = obs_tm.value[1];
        double error1 = obs_tm.value[9];

        Pose3d cameraPose0 =
            new Pose3d(
                obs_tm.value[2],
                obs_tm.value[3],
                obs_tm.value[4],
                new Rotation3d(
                    new Quaternion(
                        obs_tm.value[5], obs_tm.value[6], obs_tm.value[7], obs_tm.value[8])));
        Pose3d cameraPose1 =
            new Pose3d(
                obs_tm.value[10],
                obs_tm.value[11],
                obs_tm.value[12],
                new Rotation3d(
                    new Quaternion(
                        obs_tm.value[13], obs_tm.value[14], obs_tm.value[15], obs_tm.value[16])));

        if (error0 < error1 * 0.15) {
          cameraPose = cameraPose0;
        } else if (error1 < error0 * 0.15) {
          cameraPose = cameraPose1;
        }
      }

      if (cameraPose == null) return;

      Logger.recordOutput("Vision Think Pose pre-transform", cameraPose.toPose2d());

      cameraPose =
          cameraPose.transformBy(PoseEstimatorConstants.kCameraPositionMeters[0].inverse());

      if (cameraPose.getX() < 0
          || cameraPose.getY() < 0
          || cameraPose.getX() > Units.inchesToMeters(651.25)
          || cameraPose.getY() > Units.inchesToMeters(315.5)) return;
      // Quaternion q =
      //     new Quaternion(obs_tm.value[7], obs_tm.value[4], obs_tm.value[5], obs_tm.value[6]);
      // Pose2d pos1 =
      //     new Pose2d(obs_tm.value[1], obs_tm.value[2], new Rotation2d(new Rotation3d(q).getZ()));
      // Quaternion q2 =
      //     new Quaternion(obs_tm.value[14], obs_tm.value[11], obs_tm.value[12], obs_tm.value[13]);
      // Pose2d pos2 =
      //     new Pose2d(obs_tm.value[8], obs_tm.value[9], new Rotation2d(new
      // Rotation3d(q2).getZ()));

      // Pose2d using = m_poseEstimator.getEstimatedPosition().nearest(List.of(pos1, pos2));
      // Pose2d camera_loc = new Pose2d(SwerveConstants.kTrackWidthMeters / 2, 0, new Rotation2d());
      // using =
      //     using.transformBy(
      //         (new Transform2d(camera_loc.getTranslation(),
      // camera_loc.getRotation()).inverse()));

      // double totalDistance = 0.0;
      // for (Pose3d tagPose : tagPoses) {
      //   totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
      // }
      // double avgDistance = totalDistance / tagPoses.size();

      // System.out.print("Lodding");
      Logger.recordOutput("SwervePoseEstimator/visionEstimatedPose", cameraPose.toPose2d());
      Logger.recordOutput("SwervePoseEstimator/visionEstimatedPose3D", cameraPose);
      m_poseEstimator.addVisionMeasurement(cameraPose.toPose2d(), timestmp);
    }
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void reset(Pose2d newPos) {
    m_swerveDrive.resetGyro();
    m_poseEstimator.resetPosition(
        m_rotationSupplier.get(), m_swerveModulePositionSupplier.get(), newPos);
  }
}
