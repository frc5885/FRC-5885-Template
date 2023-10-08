// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SwerveConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {

  private final SwerveModuleIO[] m_modules = new SwerveModuleIO[4];
  private final AHRS m_gyro;
  // IO Modules can't be defined in constructor, so they are defined here
  private final SwerveModuleIOInputsAutoLogged[] m_modulesInput = {
    new SwerveModuleIOInputsAutoLogged(),
    new SwerveModuleIOInputsAutoLogged(),
    new SwerveModuleIOInputsAutoLogged(),
    new SwerveModuleIOInputsAutoLogged()
  };

  private final PIDController[] m_turnController = new PIDController[4];

  private final SwerveDrivePoseEstimator odometer;
  // private Pose2d lastPos = new Pose2d();
  private double fieldXVel = 0;
  private double fieldYVel = 0;

  private Rotation2d m_heading = new Rotation2d(0);

  /** Creates a new SwerveDrive. */
  public SwerveDrive(
      SwerveModuleIO frontLeft,
      SwerveModuleIO frontRight,
      SwerveModuleIO backLeft,
      SwerveModuleIO backRight) {

    odometer =
        new SwerveDrivePoseEstimator(
            SwerveConstants.kDriveKinematics,
            new Rotation2d(0),
            getModulePositions(),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    m_gyro = new AHRS(SPI.Port.kMXP);

    m_modules[0] = frontLeft;
    m_modules[1] = frontRight;
    m_modules[2] = backLeft;
    m_modules[3] = backRight;

    for (int i = 0; i != 4; i++) {
      m_turnController[i] = new PIDController(-0.5, 0, 0);
      m_turnController[i].enableContinuousInput(-Math.PI, Math.PI);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i != 4; i++) {
      m_modules[i].updateInputs(m_modulesInput[i]);
    }

    var chassisSpeeds = SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    double chassisRotationSpeed = chassisSpeeds.omegaRadiansPerSecond;

    if (Constants.currentMode == Mode.REAL) {
      m_heading = Rotation2d.fromDegrees(-m_gyro.getAngle());
    } else {
      m_heading = m_heading.plus(Rotation2d.fromRadians(chassisRotationSpeed * 0.02));
    }

    odometer.update(getRotation2d(), getModulePositions());

    Logger.getInstance().recordOutput("moduleStates", getModuleStates());
    Logger.getInstance().recordOutput("pos2d", odometer.getEstimatedPosition());
    Logger.getInstance().recordOutput("m_heading degrees", m_heading.getDegrees());
    Logger.getInstance().recordOutput("m_heading radians", m_heading.getRadians());
  }

  public Pose2d getFieldVelocity() {
    return new Pose2d(fieldXVel, fieldYVel, getRotation2d());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(
          m_modulesInput[0].drivePositionMeters, new Rotation2d(m_modulesInput[0].turnPositionRad)),
      new SwerveModulePosition(
          m_modulesInput[1].drivePositionMeters, new Rotation2d(m_modulesInput[1].turnPositionRad)),
      new SwerveModulePosition(
          m_modulesInput[2].drivePositionMeters, new Rotation2d(m_modulesInput[2].turnPositionRad)),
      new SwerveModulePosition(
          m_modulesInput[3].drivePositionMeters, new Rotation2d(m_modulesInput[3].turnPositionRad)),
    };
  }

  public double getHeading() {
    // return m_rotation;
    return Math.IEEEremainder(m_heading.getDegrees(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      new SwerveModuleState(
          m_modulesInput[0].driveVelocityMetersPerSec,
          new Rotation2d(m_modulesInput[0].turnPositionRad)),
      new SwerveModuleState(
          m_modulesInput[1].driveVelocityMetersPerSec,
          new Rotation2d(m_modulesInput[1].turnPositionRad)),
      new SwerveModuleState(
          m_modulesInput[2].driveVelocityMetersPerSec,
          new Rotation2d(m_modulesInput[2].turnPositionRad)),
      new SwerveModuleState(
          m_modulesInput[3].driveVelocityMetersPerSec,
          new Rotation2d(m_modulesInput[3].turnPositionRad)),
    };
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.kAttainableMaxSpeedMetersPerSecond);

    for (int i = 0; i != 4; i++) {
      if (Math.abs(desiredStates[i].speedMetersPerSecond) < 0.001) {
        m_modules[i].setDriveVoltage(0);
        m_modules[i].setTurnVoltage(0);
        continue;
      }

      desiredStates[i] =
          SwerveModuleState.optimize(
              desiredStates[i], new Rotation2d(m_modulesInput[i].turnPositionRad));

      desiredStates[i].speedMetersPerSecond *= Math.cos(m_turnController[i].getPositionError());

      m_modules[i].setDriveVoltage(
          (desiredStates[i].speedMetersPerSecond
                  / SwerveConstants.kAttainableMaxSpeedMetersPerSecond)
              * 12);

      m_modules[i].setTurnVoltage(
          (m_turnController[i].calculate(
                  m_modulesInput[i].turnPositionRad, desiredStates[i].angle.getRadians()))
              * 12);
    }
  }
}
