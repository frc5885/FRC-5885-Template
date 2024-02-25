package frc.robot.debug.PoseEstimatorSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {

  private PhotonCamera m_photonCamera;

  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private Transform3d m_robotToCam;

  private PhotonPoseEstimator m_photonPoseEstimator;

  public Vision() {
    m_photonCamera = new PhotonCamera(Constants.kCameraName);
    m_robotToCam =
        new Transform3d(
            new Translation3d(
                Constants.kCameraPositionX, Constants.kCameraPositonY, Constants.kCameraPositionZ),
            new Rotation3d(
                Constants.kCameraRoll,
                Constants.kCameraPitch,
                Constants
                    .kCameraYaw)); // Cam mounted facing forward, half a meter forward of center,
    // half a meter up from center.
    m_photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            m_photonCamera,
            m_robotToCam);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return m_photonPoseEstimator.update();
  }
}
