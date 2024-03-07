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
import edu.wpi.first.wpilibj.Timer;
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

  private double m_shooterCamPoseUpdateTimestamp = 0;
  private double m_intakeCamPoseUpdateTimestamp = 0;

  private double m_cameraPoseUpdateCooldown = 3.0; // 3 seconds
  private boolean m_shooterCheckedMostRecently = true;

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

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    // this function will continue to return a pose from one of the cameras and only switch to the other if
    // a cooldown has passed without a pose being found from the first camera
    double currentTime = Timer.getFPGATimestamp(); // Current time

    // if shooter checked most recently, check it first and only check intake if it's been long enough
    if (m_shooterCheckedMostRecently) {
      m_photonPoseEstimatorShooter.setReferencePose(prevEstimatedRobotPose);
      Optional<EstimatedRobotPose> shooterPose = m_photonPoseEstimatorShooter.update();
      // if shooter pose is still present, reset its update timestamp and return it
      if (shooterPose.isPresent()) {
        m_shooterCamPoseUpdateTimestamp = currentTime;
        return shooterPose;
      }
      // if no shooter pose, start checking intake only after cooldown
      else if (currentTime - m_intakeCamPoseUpdateTimestamp > m_cameraPoseUpdateCooldown) {
          m_photonPoseEstimatorIntake.setReferencePose(prevEstimatedRobotPose);
          Optional<EstimatedRobotPose> intakePose = m_photonPoseEstimatorIntake.update();
          // if intake pose is present, reset its update timestamp and return it
          if (intakePose.isPresent()) {
            m_intakeCamPoseUpdateTimestamp = currentTime;
            m_shooterCheckedMostRecently = false; // now intake was checked most recently
            return intakePose;
          }
        }
        // if neither pose is present (or shooter isn't present and it hasn't been long enough to switch to intake), return empty
        else {
          return Optional.empty();
        }
      }
    // if intake checked most recently, check it first and only check shooter if it's been long enough
    else {
      m_photonPoseEstimatorIntake.setReferencePose(prevEstimatedRobotPose);
      Optional<EstimatedRobotPose> intakePose = m_photonPoseEstimatorIntake.update();
      // if intake pose is still present, reset its update timestamp and return it
      if (intakePose.isPresent()) {
        m_intakeCamPoseUpdateTimestamp = currentTime;
        return intakePose;
      }
      // if no intake pose, start checking shooter only after cooldown
      else if (currentTime - m_shooterCamPoseUpdateTimestamp > m_cameraPoseUpdateCooldown) {
          m_photonPoseEstimatorShooter.setReferencePose(prevEstimatedRobotPose);
          Optional<EstimatedRobotPose> shooterPose = m_photonPoseEstimatorShooter.update();
          // if shooter pose is present, reset its update timestamp and return it
          if (shooterPose.isPresent()) {
            m_shooterCamPoseUpdateTimestamp = currentTime;
            m_shooterCheckedMostRecently = true; // now shooter was checked most recently
            return shooterPose;
          }
        }
        // if neither pose is present (or intake isn't present and it hasn't been long enough to switch to shooter), return empty
        else {
          return Optional.empty();
        }
      
      }
    // it should never reach here but vs code gets mad without this
    return Optional.empty();   
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
