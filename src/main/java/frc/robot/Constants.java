package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  /**
   * Constants related to the controller configurations. Anything that is controller specific should
   * be in here. This includes button mappings, controller ports, physical constants, power maps,
   * etc.
   */
  public static final class ControllerConstants {
    ///////////////////////////
    // Primary Drive Controller
    //
    // This controller is used for driving the robot. All
    // button remappings and other physical configs are done here.
    public static final int kDriverControllerPort = 0;
    public static final double kSwerveDriveDeadband = 0.1;

    ///////////////////////
    // Secondary Controller
    //
    // This controller is used for all other robot functions.
    // All button remappings and other physical configs are done here.
    public static final int kOperatorControllerPort = 1;
  }

  /////////////////////////
  // Swerve drive constants
  public static final class SwerveConstants {
    ////////////////
    // Driving Speed
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxSpeedAngularRadiansPerSecond = Math.PI * 1.5;

    /////////////////////////
    // Spin Correction Factor
    //
    // This value is used to offset some countersteering when turning
    // and driving at the same time. It is a guessed value but 0.075 works well.
    public static final double kDriftFactor = 0.075;

    ////////////////////////
    // Use External Encoders
    //
    // This is used to switch between using the internal NEO encoders
    // and the absolute encoders on the modules. The absolute encoders
    // are more accurate, but risk loosing connection mid match and analog
    // noise.
    public static final boolean kUseExternalEncoders = false;

    ///////////////////////////
    // Maximum Attainable Speed
    //
    // This needs to be manually found by running the robot at full speed.
    public static final double kAttainableMaxSpeedMetersPerSecond = Units.feetToMeters(15.1);

    ///////////////////////////////////
    // Robot Track Width and Base Width
    //
    // This is measured wheel to wheel. The robot *should* be a square.
    public static final double kTrackWidthMeters = Units.inchesToMeters(24.0);
    public static final double kWheelBaseWidthMeters = Units.inchesToMeters(24.0);

    //////////////////////////
    // Swerve Drive Kinematics
    // Front left, front right, rear right, rear left
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBaseWidthMeters / 2.0, kTrackWidthMeters / 2.0),
            new Translation2d(kWheelBaseWidthMeters / 2.0, -kTrackWidthMeters / 2.0),
            new Translation2d(-kWheelBaseWidthMeters / 2.0, kTrackWidthMeters / 2.0),
            new Translation2d(-kWheelBaseWidthMeters / 2.0, -kTrackWidthMeters / 2.0));

    ///////////////////
    // Module constants
    public static final class ModuleConstants {
      ///////////////////
      // Drive Motor IDs
      public static final int kLeftFrontDriveMotorID = 13;
      public static final int kRightFrontDriveMotorID = 12;
      public static final int kLeftRearDriveMotorID = 10;
      public static final int kRightRearDriveMotorID = 11;

      //////////////////////////
      // Drive Motor Directions
      public static final boolean kLeftFrontDriveMotorInverted = false;
      public static final boolean kRightFrontDriveMotorInverted = true;
      public static final boolean kLeftRearDriveMotorInverted = false;
      public static final boolean kRightRearDriveMotorInverted = true;

      /////////////////
      // Turn Motor IDs
      public static final int kLeftFrontTurnMotorID = 23;
      public static final int kRightFrontTurnMotorID = 22;
      public static final int kLeftRearTurnMotorID = 20;
      public static final int kRightRearTurnMotorID = 21;

      ////////////////////////
      // Turn Motor Directions
      public static final boolean kLeftFrontTurnMotorInverted = true;
      public static final boolean kRightFrontTurnMotorInverted = true;
      public static final boolean kLeftRearTurnMotorInverted = true;
      public static final boolean kRightRearTurnMotorInverted = true;

      ////////////////////////
      // Analog Encoders Ports
      public static final int kLeftFrontAnalogEncoderPort = 3;
      public static final int kRightFrontAnalogEncoderPort = 2;
      public static final int kLeftRearAnalogEncoderPort = 0;
      public static final int kRightRearAnalogEncoderPort = 1;

      /////////////////
      // Module Offsets
      //
      // Use SwerveGetModuleOffsets to find these values.
      public static final Rotation2d kLeftFrontModuleOffset = Rotation2d.fromDegrees(143.239);
      public static final Rotation2d kRightFrontModuleOffset = Rotation2d.fromDegrees(-15.183382);
      public static final Rotation2d kLeftRearModuleOffset = Rotation2d.fromDegrees(-141.377336);
      public static final Rotation2d kRightRearModuleOffset = Rotation2d.fromDegrees(-70.18733);

      //////////////////////
      // Drive PID Constants
      //
      // These are gussed values, changing this too much causes a lot of overshoot.
      public static final double kDriveFeedbackP = 0.1;
      public static final double kDriveFeedbackI = 0.0;
      public static final double kDriveFeedbackD = 0.0;

      ///////////////////////////////
      // Drive Feed Forward Constants
      //
      // These values are found by using the SwerveSolveFeedForward command.
      public static final double kDriveFeedForwardKs = 0.09214084677588957;
      public static final double kDriveFeedForwardKv = 2.6828478208373143;
      public static final double kDriveFeedForwardKa = 0.0;

      //////////////////////
      // Drive PID Constants
      //
      // These are gussed values.
      public static final double kTurningFeedbackP = 16.0;
      public static final double kTurningFeedbackI = 0.0;
      public static final double kTurningFeedbackD = 0.0;
      public static final double kTurningFeedbackTolerance = Units.degreesToRadians(0.65);

      //////////////////////////////////////
      // Motor Encoder Counts Per Revolution
      public static final double kDriveEncoderCPR = 42.0;
      public static final double kTurningEncoderCPR = 42.0;

      /////////////
      // Wheel Size
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

      //////////////
      // Gear Ratios
      public static final double kDriveMotorGearRatio = 1.0 / 6.75;
      public static final double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0);

      ////////////////////////////////
      // Encoder Calculation Constants
      public static final double kDriveEncoderRot2Meter =
          kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;

      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;

      //////////////////////////////
      // Module Simulation Constants
      public static final class Simulation {
        public static final double kDriveFeedbackP = 0.1;
        public static final double kDriveFeedbackI = 0.0;
        public static final double kDriveFeedbackD = 0.0;

        public static final double kDriveFeedForwardKs = 0.150551;
        public static final double kDriveFeedForwardKv = 1.30222;
        public static final double kDriveFeedForwardKa = 0.0;
      }
    }
  }
}
