// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.base.modules.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModule {

  @AutoLog
  class SwerveModuleInput {
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveTemperatureCelsius = 0.0;
    public double driveCurrent = 0.0;
    public double driveVoltage = 0.0;

    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnTemperature = 0.0;
    public double turnCurrent = 0.0;
    public double turnVoltage = 0.0;

    public double turnAbsolutePositionRad = 0.0;
  }

  default void updateInputs(SwerveModuleInput inputs) {}

  default void setDriveVoltage(double volts) {}

  default void setTurnVoltage(double volts) {}

  default void setDriveBrakeMode(boolean state) {}

  default void setTurnBrakeMode(boolean enabled) {}
}
