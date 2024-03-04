package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.base.modules.swerve.SwerveConstants;

public class AutoConstants {

  public static final class AutoStartingPositions {
    public static final Pose2d kLeftOffSubwoofer = new Pose2d(0.42, 7.0, new Rotation2d(0.0));
    public static final Pose2d kLeftOnSubwoofer = new Pose2d(0.639, 6.685, new Rotation2d(1.064));
  }

  /**
   * Constants related to the robots pose estimation. This includes camera positions, kalman filter
   * constants, and other pose estimation related constants.
   */
  public static final class PoseEstimatorConstants {
    ///////////////////
    // Camera Positions
    //
    // All positions are measured from the robots center to camera sensor, in order
    // of IDs.
    // Rotation is in degrees, relative to robot center pointing forward. Try to
    // keep this as close to 45/90/135/180/etc. as possible.
    public static final Transform3d[] kCameraPositionMeters = {
      new Transform3d(
          new Translation3d(34 / 100, Units.inchesToMeters(0), Units.inchesToMeters(0)),
          new Rotation3d(0, 0, 0)),
      //   new Transform3d(
      //       new Translation3d(
      //           Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
      //       new Rotation3d(0, 0, 0)),
    };

    //////////////////////////
    // Kalman filter constants
    //
    // Higher number means less trust in the model (more spread in the measurement)
    // Order is {x, y, theta}
    // Encoder measurements are from the encoders inside the Rev NEO motors.
    // Vision measurements are from the NoodleVision systems.
    public static final Matrix<N3, N1> kEncoderMeasurementStdDevs =
        VecBuilder.fill(0.03, 0.03, 0.03);
    public static final Matrix<N3, N1> kVisionMeasurementStdDevs =
        VecBuilder.fill(0.15, 0.15, 0.15);
  }

  // Pathplanner
  public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(SwerveConstants.Module.kDriveFeedbackP, SwerveConstants.Module.kDriveFeedbackI, SwerveConstants.Module.kDriveFeedbackI), // Translation constants 
      new PIDConstants(SwerveConstants.Module.kTurningFeedbackP, SwerveConstants.Module.kTurningFeedbackI, SwerveConstants.Module.kTurningFeedbackD), // Rotation constants 
      SwerveConstants.kMaxSpeedMetersPerSecond, 
      Math.hypot(SwerveConstants.kTrackWidthMeters/2, SwerveConstants.kWheelBaseWidthMeters/2), // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
}
