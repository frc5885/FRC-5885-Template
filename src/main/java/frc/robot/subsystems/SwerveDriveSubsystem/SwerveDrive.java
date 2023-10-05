// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final SimpleMotorFeedforward[] m_motorFeedForward = new SimpleMotorFeedforward[4];
  private final PIDController[] m_turnController = new PIDController[4];

  private final SwerveDrivePoseEstimator odometer;

  private double m_rotationRad = 0;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(
      SwerveModuleIO frontLeft,
      SwerveModuleIO frontRight,
      SwerveModuleIO backRight,
      SwerveModuleIO backLeft) {

    odometer =
        new SwerveDrivePoseEstimator(
            SwerveConstants.kDriveKinematics,
            new Rotation2d(0),
            getModulePositions(),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    m_gyro = new AHRS(SPI.Port.kMXP);

    m_modules[0] = frontLeft;
    m_modules[1] = frontRight;
    m_modules[2] = backRight;
    m_modules[3] = backLeft;

    for (int i = 0; i != 4; i++) {
      m_turnController[i] = new PIDController(0.5, 0, 0);
      m_turnController[i].enableContinuousInput(-Math.PI, Math.PI);
      m_motorFeedForward[i] = new SimpleMotorFeedforward(0.150531, 1.31831);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i != 4; i++) {
      m_modules[i].updateInputs(m_modulesInput[i]);
    }

    var chassisSpeeds = SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    m_rotationRad += chassisSpeeds.omegaRadiansPerSecond * 0.02;

    // m_rotation = -m_gyro.getAngle();
    // m_rotation += Units.radiansToDegrees(chassisRotationSpeed * 0.02);

    odometer.update(getRotation2d(), getModulePositions());

    Logger.getInstance().recordOutput("moduleStates", getModuleStates());
    Logger.getInstance().recordOutput("pos2d", odometer.getEstimatedPosition());
    Logger.getInstance().recordOutput("m_rotationSPD", chassisSpeeds.omegaRadiansPerSecond);
    Logger.getInstance().recordOutput("m_rotation", Units.degreesToRadians(m_rotationRad));
    Logger.getInstance().recordOutput("m_rotation_deg", m_rotationRad);
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
    return Math.IEEEremainder(Units.radiansToDegrees(m_rotationRad), 360);
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

      // Helps prevent optimizer from causing slip
      desiredStates[i].speedMetersPerSecond *= Math.cos(m_turnController[i].getPositionError());

      m_modules[i].setDriveVoltage(
          m_motorFeedForward[i].calculate(desiredStates[i].speedMetersPerSecond));

      m_modules[i].setTurnVoltage(
          (m_turnController[i].calculate(
                  m_modulesInput[i].turnPositionRad, desiredStates[i].angle.getRadians()))
              * 12);
    }
  }

  public void setRotationAngle(double angle) {
    m_rotationRad = Units.degreesToRadians(angle);

    for (int i = 0; i != 4; i++) {

      m_modules[i].setTurnVoltage(
          (m_turnController[i].calculate(m_modulesInput[i].turnPositionRad, angle)) * 12);
    }
  }

  public void setVoltage(double voltage) {
    for (int i = 0; i != 4; i++) {
      m_modules[i].setDriveVoltage(voltage);
    }
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public double getAverageMotorVoltage() {
    return (m_modulesInput[0].driveVoltage
            + m_modulesInput[1].driveVoltage
            + m_modulesInput[2].driveVoltage
            + m_modulesInput[3].driveVoltage)
        / 4;
  }

  public void stop() {
    for (int i = 0; i != 4; i++) {
      m_modules[i].setDriveVoltage(0);
      m_modules[i].setTurnVoltage(0);
    }
  }
}
