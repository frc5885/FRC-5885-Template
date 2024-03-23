package frc.robot;

import edu.wpi.first.math.util.Units;

public class AprilTagCameraConstants {

  public class Intake {
    public static final String kCameraName = "Arducam_OV2311_USB_Camera";
    public static final double kCameraPositionX = 0.0;
    public static final double kCameraPositonY = Units.inchesToMeters(-17.0);
    public static final double kCameraPositionZ = Units.inchesToMeters(7.5);
    public static final double kCameraRoll = 0.0;
    public static final double kCameraPitch = 0.0;
    public static final double kCameraYaw = Math.PI;
  }

  public class Shooter {
    public static final String kCameraName = "USB_Camera";
    // Higher offset = further away
    // public static final double kCameraPositionX = Units.inchesToMeters(11.56 + 0.39 + 14.0);
    public static final double kCameraPositionX = Units.inchesToMeters(11.56 + 0.39 + 12.0);
    public static final double kCameraPositonY = 0.0;
    public static final double kCameraPositionZ = Units.inchesToMeters(13.876 + 1.9);
    public static final double kCameraRoll = 0.0;
    public static final double kCameraPitch = Units.degreesToRadians(15);
    public static final double kCameraYaw = 0.0;
  }
}
