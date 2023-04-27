// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tank;

import org.littletonrobotics.junction.AutoLog;

public interface TankIO {

  @AutoLog
  public static class TankIOInputs {
    public double leftPositionMeters = 0.0;
    public double leftVelocityMetersPerSec = 0.0;
    public double rightPositionMeters = 0.0;
    public double rightVelocityMetersPerSec = 0.0;
    public double gyroYawRad = 0.0;
  }

  public default void updateInputs(TankIOInputs inputs) {}

  public default void setVoltage(double leftVolts, double rightVolts) {}

  public default void resetGyro() {}

  public default void resetEncoders() {}
}
