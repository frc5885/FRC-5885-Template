package frc.robot;

import edu.wpi.first.math.MathUtil;
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
    if (distance >= 2.6 && distance <= 2.8) {
      return (Math.atan(0.13036 / distance) + 0.290733);
    }
    return (Math.atan(0.13036 / distance) + 0.290733) * correctionFactor;
  }

  private static double getAngleFar(double distance) {
    double correctionFactor =
        SmartDashboard.getNumber(
            "WristAngleCorrectionFactorFar", Constants.kWristAngleCorrectionFactorFar);
    return MathUtil.clamp(
        (Math.atan(0.13036 / distance) + 0.290733) * correctionFactor,
        Constants.kWristEncoderMin,
        Constants.kWristEncoderMax);
  }

  public static double getVelocityFar(double distance) {
    // return (2781.37 * Math.pow(distance, 2)) + (-20080.6 * distance) + 32341.4;
    return -5000;
    // return Math.min(2520 * distance - 13457.8, Constants.kShooterMinSpeed);

    // return distance * 739.005 - 6176.93;
    // return Math.atan(-3.74952 / distance) + -3449.2;
  }

  // private static double getAnglePass(double distance) {
  //   // double correctionFactor =
  //       // SmartDashboard.getNumber(
  //       //     "WristAngleCorrectionFactorFar", Constants.kWristAngleCorrectionFactorFar);
  //   return (Math.atan(132.358 / distance) + -1.21165);
  // }
}
