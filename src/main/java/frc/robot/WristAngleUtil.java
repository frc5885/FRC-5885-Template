package frc.robot;

public class WristAngleUtil {
  public static double getAngle(double distance) {
    if (distance >= 3.1) {
      return getAngleFar(distance);
    } else {
      return getAngleClose(distance);
    }
  }

  private static double getAngleClose(double distance) {
    // double correctionFactor = 1.015;
    double correctionFactor = 1.0;
    return (0.140315 * Math.atan(1.2595 / distance) + 0.279903) * correctionFactor;
  }

  private static double getAngleFar(double distance) {
    double correctionFactor = 1.0;
    return (0.206139 * Math.atan(0.644976 / distance) + 0.29333) * correctionFactor;
  }
}
