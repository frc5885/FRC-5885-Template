package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    double correctionFactor =
        SmartDashboard.getNumber(
            "WristAngleCorrectionFactorClose", Constants.kWristAngleCorrectionFactorClose);
    return (Math.atan(0.1562 / distance) + 0.28876) * correctionFactor;
  }

  private static double getAngleFar(double distance) {
    double correctionFactor =
        SmartDashboard.getNumber(
            "WristAngleCorrectionFactorFar", Constants.kWristAngleCorrectionFactorFar);
    return (Math.atan(132.358 / distance) + -1.21165) * correctionFactor;
  }
}
