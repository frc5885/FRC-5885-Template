// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModuleSim implements SwerveModuleIO {

  private FlywheelSim m_driveMotor =
      new FlywheelSim(DCMotor.getNEO(1), SwerveConstants.Module.kDriveMotorGearRatio, 0.025);
  private FlywheelSim m_turnMotor =
      new FlywheelSim(
          DCMotor.getNEO(1), SwerveConstants.Module.kTurningMotorGearRatio, 0.004096955);

  private double m_driveVelocityMetersPerSecond;
  private double m_driveDistanceMeters;
  private double m_driveVoltage;

  private double m_turnAngleVelocityRadsPerSec;
  private double m_turnAngleRad;
  private double m_turnVoltage;

  public SwerveModuleSim(boolean isReversed) {

    m_driveDistanceMeters = 0.0;
    m_driveVelocityMetersPerSecond = 0.0;

    m_turnAngleRad = Math.random() * 2 * Math.PI;
    m_turnAngleVelocityRadsPerSec = 0.0;

    m_driveVoltage = 0.0;
    m_turnVoltage = 0.0;
  }

  public void updateInputs(SwerveModuleIOInputs inputs) {
    m_driveMotor.update(0.02);
    m_turnMotor.update(0.02);

    m_driveVelocityMetersPerSecond =
        m_driveMotor.getAngularVelocityRadPerSec() * SwerveConstants.Module.kWheelDiameterMeters;
    m_driveDistanceMeters += m_driveVelocityMetersPerSecond * 0.02;

    m_turnAngleVelocityRadsPerSec = m_turnMotor.getAngularVelocityRadPerSec();
    m_turnAngleRad += m_turnAngleVelocityRadsPerSec * 0.02;

    inputs.drivePositionMeters = m_driveDistanceMeters;
    inputs.driveVelocityMetersPerSec = m_driveVelocityMetersPerSecond;
    inputs.driveTemperatureCelsius = 0.0;
    inputs.driveCurrent = m_driveMotor.getCurrentDrawAmps();
    inputs.driveVoltage = m_driveVoltage;

    inputs.turnPositionRad = m_turnAngleRad;
    inputs.turnVelocityRadPerSec = m_turnAngleVelocityRadsPerSec;
    inputs.turnTemperature = 0.0;
    inputs.turnCurrent = m_turnMotor.getCurrentDrawAmps();
    inputs.turnVoltage = m_turnVoltage;
  }

  public void setDriveVoltage(double voltage) {
    // Simulate static voltage
    if (Math.abs(voltage) < 0.15) voltage = 0.0;
    m_driveVoltage = MathUtil.clamp(voltage, -12, 12);
    m_driveMotor.setInputVoltage(m_driveVoltage - Math.signum(voltage) * 0.15);
  }

  public void setTurnVoltage(double voltage) {
    m_turnVoltage = MathUtil.clamp(voltage, -12, 12);
    m_turnMotor.setInputVoltage(m_turnVoltage);
  }

  // Breakmode on simulator is always set to break by by default

  public void setDriveBrakeMode(boolean enabled) {}

  public void setTurnBrakeMode(boolean enabled) {}
}
