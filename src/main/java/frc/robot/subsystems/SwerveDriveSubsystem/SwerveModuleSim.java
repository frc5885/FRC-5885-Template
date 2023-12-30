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

  private FlywheelSim m_driveMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);

  private FlywheelSim m_turnMotor = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);

  private double m_driveVelocity; // m/s
  private double m_driveDistance; // m
  private double m_driveVoltage;

  private double m_turnAngleVelocity; // rad/s
  private double m_turnAngle; // rad
  private double m_turnVoltage;

  private boolean m_absoluteEncoderReversed;

  public SwerveModuleSim(boolean isReversed) {

    m_driveDistance = 0.0;
    m_driveVelocity = 0.0;

    m_turnAngle = Math.PI * 2 * Math.random();
    m_turnAngleVelocity = 0.0;

    m_driveVoltage = 0.0;
    m_turnVoltage = 0.0;

    m_absoluteEncoderReversed = isReversed;
  }

  public void updateInputs(SwerveModuleIOInputs inputs) {
    m_driveMotor.update(0.02);
    m_turnMotor.update(0.02);

    m_driveVelocity =
        m_driveMotor.getAngularVelocityRadPerSec()
            * SwerveConstants.ModuleConstants.kWheelDiameterMeters;
    m_driveDistance += m_driveVelocity * 0.02;

    m_turnAngleVelocity = m_turnMotor.getAngularVelocityRadPerSec();
    m_turnAngle += m_turnAngleVelocity * 0.02 * (m_absoluteEncoderReversed ? -1.0 : 1.0);

    inputs.drivePositionMeters = m_driveDistance;
    inputs.driveVelocityMetersPerSec = m_driveVelocity;
    inputs.driveTemperatureCelsius = 0.0;
    inputs.driveCurrent = m_driveMotor.getCurrentDrawAmps();
    inputs.driveVoltage = m_driveVoltage;

    inputs.turnPositionRad = m_turnAngle;
    inputs.turnAbsolutePositionRad = m_turnAngle;
    inputs.turnVelocityRadPerSec = m_turnAngleVelocity;
    inputs.turnTemperature = 0.0;
    inputs.turnCurrent = m_turnMotor.getCurrentDrawAmps();
    inputs.turnVoltage = m_turnVoltage;
  }

  public void setDriveVoltage(double voltage) {
    // Fake "static" friction
    voltage = MathUtil.applyDeadband(voltage / 12, 0.15 / 12) * 12;

    m_driveVoltage = MathUtil.clamp(voltage, -12, 12);
    m_driveMotor.setInputVoltage(m_driveVoltage);
  }

  public void setTurnVoltage(double voltage) {
    // Fake "static" friction
    voltage = MathUtil.applyDeadband(voltage / 12, 0.15 / 12) * 12;

    m_turnVoltage = MathUtil.clamp(voltage, -12, 12);
    m_turnMotor.setInputVoltage(m_turnVoltage);
  }

  // Breakmode on simulator is always set to break by by default

  public void setDriveBrakeMode(boolean enabled) {}

  public void setTurnBrakeMode(boolean enabled) {}
}
