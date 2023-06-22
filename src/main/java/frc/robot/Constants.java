package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final Mode currentMode = Mode.SIMULATOR;

  public static enum Mode {
    REAL,
    SIMULATOR,
    REPLAY
  }

  // Tank drive constants
  public static final class TankConstants {

    // Motors
    public static final int kLeftFrontMotorID = 1;
    public static final int kLeftRearMotorID = 2;
    public static final int kRightFrontMotorID = 3;
    public static final int kRightRearMotorID = 4;

    public static final Boolean kLeftMotorsInverted = false;
    public static final Boolean kRightMotorsInverted = false;

    // Wheel & frame
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kTrackWidthMeters = Units.inchesToMeters(31.0);

    // Encoders
    public static final int kLeftEncoderAPort = 8;
    public static final int kLeftEncoderBPort = 9;
    public static final int kRightEncoderAPort = 6;
    public static final int kRightEncoderBPort = 7;

    public static final Boolean kLeftEncoderInverted = false;
    public static final Boolean kRightEncoderInverted = false;

    public static final int kEncoderCPM = 400;
    public static final double kEncoderGearRatio = 1.0;
    public static final double kEncoderMetersPerPulse = kWheelCircumferenceMeters / kEncoderCPM;

    public static final class Auto {

      // Drive distance
      public static final double kDriveMaxVelocity = 1.75;
      public static final double kDriveMaxAcceleration = 2;

      // Auto turn
      public static final double kTurnP = 1;
      public static final double kTurnI = 0;
      public static final double kTurnD = 1;
      public static final double kTurnTolerance = 0.5;
      public static final double kTurnMaxVoltage = 7;

      // Path planning constants
      public static final double ksVolts = 0.89792;
      public static final double kvVoltSecondsPerMeter = 2.2403;
      public static final double kaVoltSecondsSquaredPerMeter = 0.60985;

      public static final double kPDriveVel = 3.2178;

      public static final double kMaxSpeedMetersPerSecond = 2;
      public static final double kMaxAccelerationMetersPerSecondSquared = 1;

      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;

      public static final double kTrackwidthMeters = 0.69;

      public static final DifferentialDriveKinematics kDriveKinematics =
          new DifferentialDriveKinematics(kTrackwidthMeters);

      // Kalman filter
      // Higher numbers means less trust

      // X, Y, Heading
      public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

      // X, Y, Heading
      public static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    }
  }

  // Swerve drive constants
  // Tank drive constants
  public static final class SwerveConstants {

    public static final double kDeadband = 0.075;

    public static final double kTrackWidth = Units.inchesToMeters(21);
    public static final double kWheelBase = Units.inchesToMeters(25.5);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final class Module {
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
      public static final double kDriveMotorGearRatio = 1 / 6.75;
      public static final double kTurningMotorGearRatio = 1 / (150.0 / 7.0);

      public static final double kDriveEncoderRot2Meter =
          kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

      public static final double kPTurning = 0.5;
    }
  }
}
