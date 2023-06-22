// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class SwerveModuleSim implements SwerveModuleIO {

  private FlywheelSim m_driveMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);

  private FlywheelSim m_turnMotor = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);

  private double m_driveVelocity; // m/s
  private double m_driveDistance; // m

  private double m_turnAngleVelocity; // rad/s
  private double m_turnAngle; // rad

  private final PIDController m_turningPidController;

  private double m_turnAbsoluteOffset;

  private boolean m_absoluteEncoderReversed;

  public SwerveModuleSim(boolean isReversed) {

    m_driveDistance = 0.0;
    m_driveVelocity = 0.0;

    m_turnAngle = 0.0;
    m_turnAngleVelocity = 0.0;

    m_turningPidController = new PIDController(0.5 * 12, 0, 0);
    m_turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    m_turnAbsoluteOffset = Math.PI * 2 * Math.random();

    m_absoluteEncoderReversed = isReversed;
    resetEncoders();
  }

  public void updateInputs() {
    m_driveMotor.update(0.02);
    m_turnMotor.update(0.02);

    m_driveVelocity = m_driveMotor.getAngularVelocityRadPerSec() * Units.inchesToMeters(1.813);
    m_driveDistance += m_driveVelocity * 0.02;

    m_turnAngleVelocity = m_turnMotor.getAngularVelocityRadPerSec();
    m_turnAngle += m_turnAngleVelocity * 0.02;
  }

  public double getDrivePosition() {
    return m_driveDistance;
  }

  public double getTurnAngle() {
    return m_turnAngle;
  }

  public double getDriveVelocity() {
    return m_driveVelocity;
  }

  public double getTurnVelcoity() {
    return m_turnAngleVelocity;
  }

  public double getAbsoluteEncoderRad() {
    double ang = ((getTurnAngle() % Math.PI) + Math.PI - m_turnAbsoluteOffset);
    return ang * (m_absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void setDriveVoltage(double voltage) {
    m_driveMotor.setInputVoltage(voltage);
  }

  public void setTurnVoltage(double voltage) {
    m_turnMotor.setInputVoltage(voltage);
  }

  public void resetEncoders() {
    m_driveDistance = 0;
    m_turnAngle = getAbsoluteEncoderRad();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurnAngle()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);

    setDriveVoltage((state.speedMetersPerSecond / 4.125) * 12);
    setTurnVoltage(m_turningPidController.calculate(getTurnAngle(), state.angle.getRadians()));
  }

  public void stop() {
    m_driveMotor.setInputVoltage(0);
    m_turnMotor.setInputVoltage(0);
  }
}
