package frc.robot;

public final class Constants {
  // Intake Motor IDs
  public static final int kIntakeLeft = 31;
  public static final int kIntakeRight = 30;
  public static final int kBeambreak = 6;
  public static final int kFeeder = 40;
  public static final int kShooterTop = 50;
  public static final int kShooterBottom = 51;
  public static final int kArm = 60;
  public static final int kWrist = 61;
  public static final int kClimberLeft = 55;
  public static final int kClimberRight = 56;

  // Arm Encoder Stuff
  public static final double kArmEncoderMax = 1.0;
  public static final double kArmEncoderMin = 0.0;
  public static final double kArmStow = 0.0;
  public static final double kArmAmp = 1000.0;

  // Wrist Encoder Stuff
  public static final double kWristEncoderMax = 1.0;
  public static final double kWristEncoderMin = 0.0;
  public static final double kWristStow = 0.0;
  public static final double kWristAmp = 0.0;

  // Climber Deadzones
  public static final double kOperatorRightDeadzone = 0.02;
  public static final double kOperatorLeftDeadzone = 0.02;
  
}
