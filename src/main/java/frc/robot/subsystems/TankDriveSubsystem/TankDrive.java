// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.TankDriveSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TankDrive extends SubsystemBase {

  private final TankDriveIO m_io;
  private final TankDriveIOInputsAutoLogged m_inputs = new TankDriveIOInputsAutoLogged();
  private double m_relativeDistanceMeters;

  /** Creates a new Tank. */
  public TankDrive(TankDriveIO io) {
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Tank Drive", m_inputs);
  }

  public void driveTankVolts(double leftSpeed, double rightSpeed) {
    m_io.setVoltage(leftSpeed, rightSpeed);
  }

  public void driveTank(double leftSpeed, double rightSpeed, double voltageMax) {
    final double deadzone = 0.075;

    m_io.setVoltage(
        (Math.abs(leftSpeed) < deadzone)
            ? 0
            : (leftSpeed * voltageMax) * (1 / (1 - deadzone)) - deadzone,
        (Math.abs(rightSpeed) < deadzone)
            ? 0
            : (rightSpeed * voltageMax) * (1 / (1 - deadzone)) - deadzone);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_inputs.leftVelocityMetersPerSec, m_inputs.rightVelocityMetersPerSec);
  }

  public void setVoltage(double leftVoltage, double rightVoltage) {
    m_io.setVoltage(leftVoltage, rightVoltage);
  }

  public double getStoredDistanceMeters() {
    return m_relativeDistanceMeters;
  }

  public void storeCurrentAverageDistance() {
    m_relativeDistanceMeters = getAveragePositionMeters();
  }

  public void stop() {
    m_io.setVoltage(0, 0);
  }

  public Rotation2d getRotation() {
    return m_inputs.gyroRotation;
  }

  public void resetEncoders() {
    m_io.resetEncoders();
  }

  public double getAveragePositionMeters() {
    return (m_inputs.leftPositionMeters + m_inputs.rightPositionMeters) / 2.0;
  }

  public double getLeftPositionMeters() {
    return m_inputs.leftPositionMeters;
  }

  public double getRightPositionMeters() {
    return m_inputs.rightPositionMeters;
  }

  public double getLeftVelocityMeters() {
    return m_inputs.leftVelocityMetersPerSec;
  }

  public double getrightVelocityMeters() {
    return m_inputs.rightVelocityMetersPerSec;
  }
}
