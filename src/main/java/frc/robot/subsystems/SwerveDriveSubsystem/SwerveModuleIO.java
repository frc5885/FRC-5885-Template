// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

  @AutoLog
  public static class SwerveModuleIOInputs {
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveTemperature = 0.0;
    public double driveCurrent = 0.0;
    public double driveBusVoltage = 0.0;

    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnTemperature = 0.0;
    public double turnCurrent = 0.0;
    public double turnBusVoltage = 0.0;
  }

  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  public default void setDriveVoltage(double volts) {}

  public default void setTurnVoltage(double volts) {}

  // public default double getDrivePosition() {
  // }

  // public default double getTurningPosition() {
  // }

  // public default double getDriveVelocity() {
  // }

  // public default double getTurningVelocity() {
  // }
}
