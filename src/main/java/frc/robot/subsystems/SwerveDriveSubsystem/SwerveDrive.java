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
import frc.robot.Constants.SwerveConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {

  // SwerveModuleIO m_frontLeft = new SwerveModuleNEO(13, 23, 3, new
  // Rotation2d(2.50), false, false);
  // SwerveModuleIO m_frontRight = new SwerveModuleNEO(12, 22, 2, new
  // Rotation2d(-0.265), false, true);
  // SwerveModuleIO m_backLeft = new SwerveModuleNEO(10, 20, 0, new
  // Rotation2d(-2.4675), false, false);
  // SwerveModuleIO m_backRight = new SwerveModuleNEO(11, 21, 1, new
  // Rotation2d(-1.225), false, true);

  // SwerveModuleIOInputsAutoLogged m_frontLeftInputs = new
  // SwerveModuleIOInputsAutoLogged();
  // SwerveModuleIOInputsAutoLogged m_frontRightInputs = new
  // SwerveModuleIOInputsAutoLogged();
  // SwerveModuleIOInputsAutoLogged m_backLeftInputs = new
  // SwerveModuleIOInputsAutoLogged();
  // SwerveModuleIOInputsAutoLogged m_backRightInputs = new
  // SwerveModuleIOInputsAutoLogged();

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

  private double m_rotation = 0;

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
      m_turnController[i] = new PIDController(0.1, 0, 0);
      m_turnController[i].enableContinuousInput(-Math.PI, Math.PI);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i != 4; i++) {
      m_modules[i].updateInputs(m_modulesInput[i]);
    }

    // var chassisSpeeds =
    // SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    // double chassisRotationSpeed = chassisSpeeds.omegaRadiansPerSecond;

    m_rotation = m_gyro.getAngle();
    // m_rotation += chassisRotationSpeed * 0.02;

    odometer.update(getRotation2d(), getModulePositions());

    Logger.getInstance().recordOutput("moduleStates", getModuleStates());
    Logger.getInstance().recordOutput("pos2d", odometer.getEstimatedPosition());
    Logger.getInstance().recordOutput("m_rotation", m_rotation);
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
    return Math.IEEEremainder(m_rotation, 360);
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
        continue;
      }

      desiredStates[i] =
          SwerveModuleState.optimize(
              desiredStates[i], new Rotation2d(m_modulesInput[i].turnPositionRad));

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
