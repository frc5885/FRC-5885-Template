// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {

  SwerveModuleSim m_frontLeft = new SwerveModuleSim(false);
  SwerveModuleSim m_frontRight = new SwerveModuleSim(false);
  SwerveModuleSim m_backLeft = new SwerveModuleSim(false);
  SwerveModuleSim m_backRight = new SwerveModuleSim(false);

  private final SwerveDrivePoseEstimator odometer;
  private Pose2d lastPos = new Pose2d();
  private double fieldXVel = 0;
  private double fieldYVel = 0;

  private double m_rotation = 0;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    odometer =
        new SwerveDrivePoseEstimator(
            SwerveConstants.kDriveKinematics,
            new Rotation2d(0),
            getModulePositions(),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public Pose2d getFieldVelocity() {
    return new Pose2d(fieldXVel, fieldYVel, getRotation2d());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  public double getHeading() {
    return Math.IEEEremainder(m_rotation, 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState()
    };
  }

  @Override
  public void periodic() {
    m_frontLeft.updateInputs();
    m_frontRight.updateInputs();
    m_backLeft.updateInputs();
    m_backRight.updateInputs();

    var chassisSpeeds = SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    double chassisRotationSpeed = chassisSpeeds.omegaRadiansPerSecond;

    m_rotation += Units.radiansToDegrees(chassisRotationSpeed * 0.02);

    odometer.update(getRotation2d(), getModulePositions());

    Logger.getInstance().recordOutput("moduleStates", getModuleStates());
    Logger.getInstance().recordOutput("pos2d", odometer.getEstimatedPosition());

    // fieldXVel = (odometer.getEstimatedPosition().getX() - lastPos.getX()) / 0.02;
    // fieldYVel = (odometer.getEstimatedPosition().getY() - lastPos.getY()) / 0.02;

    lastPos = odometer.getEstimatedPosition();
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 4.125);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }
}
