package frc.robot;

public class WristAngleUtil {
  public static double getAngle(double distance) {
    if (distance >= 2.0) {
      return getAngleFar(distance);
    } else {
      return getAngleClose(distance);
    }
  }

  private static double getAngleClose(double distance) {
    double correctionFactor = 1.0;
    return (0.07356 * Math.atan(2.00 / distance) + 0.2927) * correctionFactor;
  }

  private static double getAngleFar(double distance) {
    double correctionFactor = 1.0;
    return (0.07356 * Math.atan(2.00 / distance) + 0.2927) * correctionFactor;
  }
}
