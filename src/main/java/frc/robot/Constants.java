package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final class SwerveConstants {

    // Motors
    public static final int kLeftFrontDriveMotorID = 13;
    public static final int kRightFrontDriveMotorID = 12;
    public static final int kLeftRearDriveMotorID = 11;
    public static final int kRightRearDriveMotorID = 10;

    public static final boolean kLeftFrontDriveMotorInverted = false;
    public static final boolean kRightFrontDriveMotorInverted = true;
    public static final boolean kLeftRearDriveMotorInverted = true;
    public static final boolean kRightRearDriveMotorInverted = false;

    public static final int kLeftFrontTurnMotorID = 23;
    public static final int kRightFrontTurnMotorID = 22;
    public static final int kLeftRearTurnMotorID = 21;
    public static final int kRightRearTurnMotorID = 20;

    public static final boolean kLeftFrontTurnMotorInverted = false;
    public static final boolean kRightFrontTurnMotorInverted = false;
    public static final boolean kLeftRearTurnMotorInverted = false;
    public static final boolean kRightRearTurnMotorInverted = false;

    // Module offsets
    public static final Rotation2d kLeftFrontModuleOffset = new Rotation2d(2.50);
    public static final Rotation2d kRightFrontModuleOffset = new Rotation2d(-0.265);
    public static final Rotation2d kLeftRearModuleOffset = new Rotation2d(-1.225);
    public static final Rotation2d kRightRearModuleOffset = new Rotation2d(-2.4675);

    // Analog encoders
    public static final int kLeftFrontAnalogEncoderPort = 3;
    public static final int kRightFrontAnalogEncoderPort = 2;
    public static final int kLeftRearAnalogEncoderPort = 1;
    public static final int kRightRearAnalogEncoderPort = 0;

    // Controller

    public static final double kDeadband = 0.08;

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
