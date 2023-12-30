// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
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
  private final PIDController[] m_driveController = new PIDController[4];
  private final SimpleMotorFeedforward[] m_driveFeedforward = new SimpleMotorFeedforward[4];

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

    m_gyro = new AHRS(SPI.Port.kMXP);
    resetGyro();

    m_modules[0] = frontLeft;
    m_modules[1] = frontRight;
    m_modules[2] = backLeft;
    m_modules[3] = backRight;

    for (int i = 0; i != 4; i++) {
      m_turnController[i] =
          new PIDController(
              ModuleConstants.kTurningFeedbackP,
              ModuleConstants.kTurningFeedbackI,
              ModuleConstants.kTurningFeedbackD);
      m_turnController[i].enableContinuousInput(-Math.PI, Math.PI);

      if (Constants.kCurrentMode == Mode.REAL) {
        m_driveController[i] =
            new PIDController(
                ModuleConstants.kDriveFeedbackP,
                ModuleConstants.kDriveFeedbackI,
                ModuleConstants.kDriveFeedbackD);
        m_driveFeedforward[i] =
            new SimpleMotorFeedforward(
                ModuleConstants.kDriveFeedForwardKs,
                ModuleConstants.kDriveFeedForwardKv,
                ModuleConstants.kDriveFeedForwardKa);
      } else {
        m_driveController[i] =
            new PIDController(
                ModuleConstants.Simulation.kDriveFeedbackP,
                ModuleConstants.Simulation.kDriveFeedbackI,
                ModuleConstants.Simulation.kDriveFeedbackD);
        m_driveFeedforward[i] =
            new SimpleMotorFeedforward(
                ModuleConstants.Simulation.kDriveFeedForwardKs,
                ModuleConstants.Simulation.kDriveFeedForwardKv,
                ModuleConstants.Simulation.kDriveFeedForwardKa);
      }
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i != 4; i++) {
      m_modules[i].updateInputs(m_modulesInput[i]);
      Logger.processInputs("SwerveDrive/Modules/Module" + Integer.toString(i), m_modulesInput[i]);
    }

    var chassisSpeeds = SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    double chassisRotationSpeed = chassisSpeeds.omegaRadiansPerSecond;

    if (Constants.kCurrentMode == Mode.REAL) {
      m_heading = Rotation2d.fromDegrees(-m_gyro.getAngle());
    } else {
      m_heading = m_heading.plus(Rotation2d.fromRadians(chassisRotationSpeed * 0.02));
    }

    Logger.recordOutput("SwerveDrive/currentModuleStates", getModuleStates());
    Logger.recordOutput("SwerveDrive/headingDegrees", m_heading.getDegrees());
    Logger.recordOutput("SwerveDrive/headingRadians", m_heading.getRadians());
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
    return m_heading;
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

      // In case of a sharp wheel turn, this helps prevent the
      // innertia of the robot from sliding too much.
      desiredStates[i].speedMetersPerSecond *= Math.cos(m_turnController[i].getPositionError());

      m_modules[i].setDriveVoltage(
          m_driveFeedforward[i].calculate(desiredStates[i].speedMetersPerSecond)
              + m_driveController[i].calculate(
                  m_modulesInput[i].driveVelocityMetersPerSec,
                  desiredStates[i].speedMetersPerSecond));

      m_modules[i].setTurnVoltage(
          (m_turnController[i].calculate(
              m_modulesInput[i].turnPositionRad, desiredStates[i].angle.getRadians())));
    }
  }

  public void resetGyro() {
    m_gyro.reset();
    m_heading = Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public void setModulesAngle(double angle) {

    for (int i = 0; i != 4; i++) {
      m_modules[i].setTurnVoltage(
          (m_turnController[i].calculate(m_modulesInput[i].turnPositionRad, angle)));
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

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

  public double getAverageMotorVoltage() {
    return (m_modulesInput[0].driveVoltage
            + m_modulesInput[1].driveVoltage
            + m_modulesInput[2].driveVoltage
            + m_modulesInput[3].driveVoltage)
        / 4;
  }

  public double getAngularVelocity() {
    return getChassisSpeeds().omegaRadiansPerSecond;
  }

  public void stop() {
    for (int i = 0; i != 4; i++) {
      m_modules[i].setDriveVoltage(0);
      m_modules[i].setTurnVoltage(0);
    }
  }
}
