package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final Mode currentMode = Mode.SIMULATOR;

  public static enum Mode {
    REAL,
    SIMULATOR,
    REPLAY
  }

  // Swerve drive constants
  // Tank drive constants
  public static final class SwerveConstants {

    // Motors
    public static final int kLeftFrontMotorID = 1;
    public static final int kLeftRearMotorID = 2;
    public static final int kRightFrontMotorID = 3;
    public static final int kRightRearMotorID = 4;

    public static final Boolean kLeftMotorsInverted = false;
    public static final Boolean kRightMotorsInverted = false;

    // Controller

    public static final double kDeadband = 0.075;

    //

    public static final double kTrackWidth = Units.inchesToMeters(25.5);
    public static final double kWheelBase = Units.inchesToMeters(25.5);

    public static final double kAttainableMaxSpeedMetersPerSecond = Units.feetToMeters(14.5);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final class Module {
      public static final double kDriveEncoderCPR = 42.0;
      public static final double kTurningEncoderCPR = 42.0;
      public static final double kRelativeEncoderCPR = 4096.0;

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
