package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final Mode kCurrentMode = Mode.REAL;

  public static enum Mode {
    REAL,
    SIMULATOR,
    REPLAY
  }

  // Controller constants
  public static final class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // Controller
    public static final double kDeadband = 0.12;
  }

  public static final class PoseEstimatorConstants {

    // Camera Positions
    // Relative to robot center to camera sensor, in order of IDs
    // Rotation is in degrees, relative to robot center pointing forward
    public static final Transform3d[] kCameraPositionMeters = {
      new Transform3d(
          new Translation3d(34 / 100, Units.inchesToMeters(0), Units.inchesToMeters(0)),
          new Rotation3d(0, 0, 0)),
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
          new Rotation3d(0, 0, 0)),
    };

    // Kalman filter constants
    // Higher number means less trust in the model
    // Order is {x, y, theta}
    public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.03, 0.03, 0.03);
    public static final Matrix<N3, N1> kVisionMeasurementStdDevs =
        VecBuilder.fill(0.15, 0.15, 0.15);
  }

  // Swerve drive constants
  public static final class SwerveConstants {

    // Speed
    public static final double kMaxSpeedXMetersPerSecond = 2;
    public static final double kMaxSpeedYMetersPerSecond = 2;
    public static final double kMaxSpeedAngularRadiansPerSecond = Math.PI * 1;

    public static final double kMaxAccelerationXMetersPerSecondSquared = 2.0;
    public static final double kMaxAccelerationYMetersPerSecondSquared = 2.0;
    public static final double kMaxAccelerationAngularRadiansPerSecondSquared = Math.PI * 3.5;

    // Motors
    public static final int kLeftFrontDriveMotorID = 13;
    public static final int kRightFrontDriveMotorID = 12;
    public static final int kLeftRearDriveMotorID = 10;
    public static final int kRightRearDriveMotorID = 11;

    public static final boolean kLeftFrontDriveMotorInverted = false;
    public static final boolean kRightFrontDriveMotorInverted = true;
    public static final boolean kLeftRearDriveMotorInverted = false;
    public static final boolean kRightRearDriveMotorInverted = true;

    public static final int kLeftFrontTurnMotorID = 23;
    public static final int kRightFrontTurnMotorID = 22;
    public static final int kLeftRearTurnMotorID = 20;
    public static final int kRightRearTurnMotorID = 21;

    public static final boolean kLeftFrontTurnMotorInverted = true;
    public static final boolean kRightFrontTurnMotorInverted = true;
    public static final boolean kLeftRearTurnMotorInverted = true;
    public static final boolean kRightRearTurnMotorInverted = true;

    // Module offsets
    public static final Rotation2d kLeftFrontModuleOffset = Rotation2d.fromDegrees(143.239);
    public static final Rotation2d kRightFrontModuleOffset = Rotation2d.fromDegrees(-15.183382);
    public static final Rotation2d kLeftRearModuleOffset = Rotation2d.fromDegrees(-141.377336);
    public static final Rotation2d kRightRearModuleOffset = Rotation2d.fromDegrees(-70.18733);

    // Analog encoders
    public static final int kLeftFrontAnalogEncoderPort = 3;
    public static final int kRightFrontAnalogEncoderPort = 2;
    public static final int kLeftRearAnalogEncoderPort = 0;
    public static final int kRightRearAnalogEncoderPort = 1;

    // Physical constants

    public static final double kAttainableMaxSpeedMetersPerSecond = Units.feetToMeters(14.5);

    public static final double kTrackWidthMeters = Units.inchesToMeters(25.5);
    public static final double kWheelBaseMeters = Units.inchesToMeters(25.5);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
            new Translation2d(kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0),
            new Translation2d(-kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
            new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0));

    public static final double kFeedForwardKs = 0.09214084677588957;
    public static final double kFeedForwardKv = 2.6828478208373143;
    public static final double kFeedForwardKa = 0.0;

    // Module constants

    public static final class Module {
      public static final double kDriveEncoderCPR = 42.0;
      public static final double kTurningEncoderCPR = 42.0;
      public static final double kRelativeEncoderCPR = 4096.0;

      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
      public static final double kDriveMotorGearRatio = 1.0 / 6.75;
      public static final double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0);

      public static final double kDriveEncoderRot2Meter =
          kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;

      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;

      public static final double kPTurning = 0.5;
    }

    public static final class Simulation {
      public static final double kFeedForwardKs = 0.150551;
      public static final double kFeedForwardKv = 1.30222;
      public static final double kFeedForwardKa = 0.0;
    }
  }
}
