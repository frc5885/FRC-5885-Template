package frc.robot.subsystems.PoseEstimatorSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTagCameraConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// this object is created in the WCRobot class
public class PhotonVisionSystem extends SubsystemBase {

  private PhotonCamera m_photonCamera;

  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private Transform3d m_robotToCam;

  private PhotonPoseEstimator m_photonPoseEstimator;

  public PhotonVisionSystem() {
    m_photonCamera = new PhotonCamera(AprilTagCameraConstants.kCameraName);
    m_robotToCam =
        new Transform3d(
            new Translation3d(
                AprilTagCameraConstants.kCameraPositionX,
                AprilTagCameraConstants.kCameraPositonY,
                AprilTagCameraConstants.kCameraPositionZ),
            new Rotation3d(
                AprilTagCameraConstants.kCameraRoll,
                AprilTagCameraConstants.kCameraPitch,
                AprilTagCameraConstants.kCameraYaw));
    m_photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            m_photonCamera,
            m_robotToCam);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    System.out.println("UpdatedCameraPose");
    m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return m_photonPoseEstimator.update();
  }

  // can add other functions here later to check for specific april tags I think

  // this one might get the angle up to april tag ID 7 (middle of blue speaker) but idk
  public Optional<Double> GetYawToSpeaker() {
    PhotonPipelineResult result = m_photonCamera.getLatestResult();
    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      // check each result for the tag ID
      for (int i = 0; i < targets.size(); i++) {
        if (targets.get(i).getFiducialId() == 7) {
          // do something with the angle
          double angle = targets.get(i).getPitch();
          return Optional.of(angle);
        }
      }
    }
    return Optional.empty();
  }
}
