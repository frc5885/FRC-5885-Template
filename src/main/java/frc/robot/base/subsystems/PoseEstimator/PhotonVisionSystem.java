package frc.robot.base.subsystems.PoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTagCameraConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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
    // System.out.println("UpdatedCameraPose");
    m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return m_photonPoseEstimator.update();
  }

  // can add other functions here later to check for specific april tags I think

  // this one might get the angle up to april tag ID 7 (middle of blue speaker) but idk
  public double getAngleToTarget(Pose2d robotPose, int targetID) {
    Pose2d aprilTagLocation = aprilTagFieldLayout.getTagPose(targetID).get().toPose2d();
    double targetX = aprilTagLocation.getTranslation().getX();
    double targetY = aprilTagLocation.getTranslation().getY();
    double robotX = robotPose.getTranslation().getX();
    double robotY = robotPose.getTranslation().getY();
    double angleToTarget = Math.atan2(targetY - robotY, targetX - robotX);
    SmartDashboard.putNumber("AngleToTarget", angleToTarget);
    return angleToTarget;
  }

  public double distanceToTarget(Pose2d robotPose, int targetID) {
    Pose2d aprilTagLocation = aprilTagFieldLayout.getTagPose(targetID).get().toPose2d();
    double targetX = aprilTagLocation.getTranslation().getX();
    double targetY = aprilTagLocation.getTranslation().getY();
    double robotX = robotPose.getTranslation().getX();
    double robotY = robotPose.getTranslation().getY();
    double distanceToTarget =
        Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2));
    SmartDashboard.putNumber("DistanceToTarget", distanceToTarget);
    return distanceToTarget;
  }
}
