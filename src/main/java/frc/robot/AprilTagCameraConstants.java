package frc.robot;

import edu.wpi.first.math.util.Units;

public class AprilTagCameraConstants {

  public class Intake {
    public static final String kCameraName = "Arducam_5MP_Camera_Module";
  }

  public class ShooterBlue {
    public static final String kCameraName = "USB_Camera";
    // Higher offset = further away
    // Blue
    public static final double kCameraPositionX = Units.inchesToMeters(11.56 + 0.39 + 13.0);
    // Red
    // public static final double kCameraPositionX = Units.inchesToMeters(11.56 + 0.39 + 12.0);
    public static final double kCameraPositonY = 0.0;
    public static final double kCameraPositionZ = Units.inchesToMeters(13.876 + 1.9);
    public static final double kCameraRoll = 0.0;
    public static final double kCameraPitch = Units.degreesToRadians(15);
    public static final double kCameraYaw = 0.0;
  }

  public class ShooterRed {
    public static final String kCameraName = "USB_Camera";
    // Higher offset = further away
    // Blue
    // public static final double kCameraPositionX = Units.inchesToMeters(11.56 + 0.39 + 14.0);
    // Red
    public static final double kCameraPositionX = Units.inchesToMeters(11.56 + 0.39 + 13.0);
    public static final double kCameraPositonY = 0.0;
    public static final double kCameraPositionZ = Units.inchesToMeters(13.876 + 1.9);
    public static final double kCameraRoll = 0.0;
    public static final double kCameraPitch = Units.degreesToRadians(15);
    public static final double kCameraYaw = 0.0;
  }
}
