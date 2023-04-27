// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {

  private final SwerveModuleIO m_moduleTopLeft;
  private final SwerveModuleIOInputsAutoLogged m_inputsTopLeft =
      new SwerveModuleIOInputsAutoLogged();

  private final SwerveModuleIO m_moduleTopRight;
  private final SwerveModuleIOInputsAutoLogged m_inputsTopRight =
      new SwerveModuleIOInputsAutoLogged();

  private final SwerveModuleIO m_moduleBottomLeft;
  private final SwerveModuleIOInputsAutoLogged m_inputsBottomLeft =
      new SwerveModuleIOInputsAutoLogged();

  private final SwerveModuleIO m_moduleBottomRight;
  private final SwerveModuleIOInputsAutoLogged m_inputsBottomRight =
      new SwerveModuleIOInputsAutoLogged();

  private final AHRS m_gyro;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(
      SwerveModuleIO moduleTopLeft,
      SwerveModuleIO moduleTopRight,
      SwerveModuleIO moduleBottomLeft,
      SwerveModuleIO moduleBottomRight) {
    m_moduleTopLeft = moduleTopLeft;
    m_moduleTopRight = moduleTopRight;
    m_moduleBottomLeft = moduleBottomLeft;
    m_moduleBottomRight = moduleBottomRight;

    m_gyro = new AHRS(SPI.Port.kMXP);

    // Wait until we are done calibrating, then zero the heading
    new Thread(
            () -> {
              while (m_gyro.isCalibrating()) {}
              zeroHeading();
            })
        .start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules() {
    m_moduleTopLeft.stop();
    m_moduleTopRight.stop();
    m_moduleBottomLeft.stop();
    m_moduleBottomRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, SwerveConstants.Module.kPhysicalMaxSpeedMetersPerSecond);
    m_moduleTopLeft.setDesiredState(states[0]);
    m_moduleTopRight.setDesiredState(states[1]);
    m_moduleBottomLeft.setDesiredState(states[2]);
    m_moduleBottomRight.setDesiredState(states[3]);
  }
}
