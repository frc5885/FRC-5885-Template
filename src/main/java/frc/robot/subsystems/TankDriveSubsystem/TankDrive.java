// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.TankDriveSubsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TankConstants;
import org.littletonrobotics.junction.Logger;

public class TankDrive extends SubsystemBase {

  private final TankDriveIO m_io;
  private final TankDriveIOInputsAutoLogged m_inputs = new TankDriveIOInputsAutoLogged();
  private final DifferentialDrivePoseEstimator m_odometry =
      new DifferentialDrivePoseEstimator(
          TankConstants.Auto.kDriveKinematics,
          new Rotation2d(0),
          0,
          0,
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  private double m_relativeDistanceMeters;

  /** Creates a new Tank. */
  public TankDrive(TankDriveIO io) {
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Tank", m_inputs);

    // Update odometry and log the new pose
    m_odometry.update(
        new Rotation2d(m_inputs.gyroYawRad),
        m_inputs.leftPositionMeters,
        m_inputs.rightPositionMeters);
    Logger.getInstance().recordOutput("Odometry", m_odometry.getEstimatedPosition());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        new Rotation2d(m_inputs.gyroYawRad),
        m_inputs.leftPositionMeters,
        m_inputs.rightPositionMeters,
        pose);
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

  public double getHeadingDegrees() {
    return Units.radiansToDegrees(m_inputs.gyroYawRad);
  }

  public void resetEncoders() {
    m_io.resetEncoders();
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
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
