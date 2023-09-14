// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class MathTools {
  public static double clamp(double input, double min, double max) {
    return Math.min(max, Math.max(input, min));
  }
}
