package frc.robot.base.subsystems.PoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  private PhotonCamera m_photonCameraIntake;
  private PhotonCamera m_photonCameraShooter;

  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private Transform3d m_robotToCamIntake;
  private Transform3d m_robotToCamShooter;

  private PhotonPoseEstimator m_photonPoseEstimatorIntake;
  private PhotonPoseEstimator m_photonPoseEstimatorShooter;

  public PhotonVisionSystem() {
    try {
      m_photonCameraIntake = new PhotonCamera(AprilTagCameraConstants.Intake.kCameraName);
      m_photonCameraShooter = new PhotonCamera(AprilTagCameraConstants.Shooter.kCameraName);
    } catch (Exception e) {
      System.out.println("Photon camera not found: " + e.getMessage());
      return; // Exit the constructor if the camera isn't found
    }
    m_robotToCamIntake =
        new Transform3d(
            new Translation3d(
                AprilTagCameraConstants.Intake.kCameraPositionX,
                AprilTagCameraConstants.Intake.kCameraPositonY,
                AprilTagCameraConstants.Intake.kCameraPositionZ),
            new Rotation3d(
                AprilTagCameraConstants.Intake.kCameraRoll,
                AprilTagCameraConstants.Intake.kCameraPitch,
                AprilTagCameraConstants.Intake.kCameraYaw));
    m_robotToCamShooter =
        new Transform3d(
            new Translation3d(
                AprilTagCameraConstants.Shooter.kCameraPositionX,
                AprilTagCameraConstants.Shooter.kCameraPositonY,
                AprilTagCameraConstants.Shooter.kCameraPositionZ),
            new Rotation3d(
                AprilTagCameraConstants.Shooter.kCameraRoll,
                AprilTagCameraConstants.Shooter.kCameraPitch,
                AprilTagCameraConstants.Shooter.kCameraYaw));
    m_photonPoseEstimatorIntake =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            m_photonCameraIntake,
            m_robotToCamIntake);
    m_photonPoseEstimatorShooter =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            m_photonCameraShooter,
            m_robotToCamShooter);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseIntake(Pose2d prevEstimatedRobotPose) {
    // System.out.println("UpdatedCameraPose");
    m_photonPoseEstimatorIntake.setReferencePose(prevEstimatedRobotPose);
    return m_photonPoseEstimatorIntake.update();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseShooter(Pose2d prevEstimatedRobotPose) {
    // System.out.println("UpdatedCameraPose");
    m_photonPoseEstimatorShooter.setReferencePose(prevEstimatedRobotPose);
    return m_photonPoseEstimatorShooter.update();
  }

  public int getTargetID() {
    // returns the April Tag ID of the speaker for either the blue or red alliance
    // blue alliance speaker is 7, red alliance speaker is 4
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 7 : 4;
  }

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
