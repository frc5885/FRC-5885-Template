// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {

  SwerveModuleIO m_frontLeft = new SwerveModuleNEO(13, 23, 3, new Rotation2d(2.50), false, false);
  SwerveModuleIO m_frontRight = new SwerveModuleNEO(12, 22, 2, new Rotation2d(-0.265), false, true);
  SwerveModuleIO m_backLeft = new SwerveModuleNEO(10, 20, 0, new Rotation2d(-2.4675), false, false);
  SwerveModuleIO m_backRight = new SwerveModuleNEO(11, 21, 1, new Rotation2d(-1.225), false, true);

  SwerveModuleIOInputsAutoLogged m_frontLeftInputs = new SwerveModuleIOInputsAutoLogged();
  SwerveModuleIOInputsAutoLogged m_frontRightInputs = new SwerveModuleIOInputsAutoLogged();
  SwerveModuleIOInputsAutoLogged m_backLeftInputs = new SwerveModuleIOInputsAutoLogged();
  SwerveModuleIOInputsAutoLogged m_backRightInputs = new SwerveModuleIOInputsAutoLogged();

  private final PIDController[] m_turnController = new PIDController[4];

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

    for (int i = 0; i != 4; i++) {
      m_turnController[i] = new PIDController(-0.1 * 12, 0, 0);
      m_turnController[i].enableContinuousInput(-Math.PI, Math.PI);
    }
  }

  @Override
  public void periodic() {
    m_frontLeft.updateInputs(m_frontLeftInputs);
    m_frontRight.updateInputs(m_frontRightInputs);
    m_backLeft.updateInputs(m_backLeftInputs);
    m_backRight.updateInputs(m_backRightInputs);

    var chassisSpeeds = SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    double chassisRotationSpeed = chassisSpeeds.omegaRadiansPerSecond;

    m_rotation += 0; // Units.radiansToDegrees(chassisRotationSpeed * 0.02);

    odometer.update(getRotation2d(), getModulePositions());

    Logger.getInstance().recordOutput("moduleStates", getModuleStates());
    Logger.getInstance().recordOutput("pos2d", odometer.getEstimatedPosition());

    // fieldXVel = (odometer.getEstimatedPosition().getX() - lastPos.getX()) / 0.02;
    // fieldYVel = (odometer.getEstimatedPosition().getY() - lastPos.getY()) / 0.02;

    lastPos = odometer.getEstimatedPosition();
  }

  public Pose2d getFieldVelocity() {
    return new Pose2d(fieldXVel, fieldYVel, getRotation2d());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(
          m_frontLeftInputs.turnPositionRad, new Rotation2d(m_frontLeftInputs.drivePositionMeters)),
      new SwerveModulePosition(
          m_frontRightInputs.turnPositionRad,
          new Rotation2d(m_frontRightInputs.drivePositionMeters)),
      new SwerveModulePosition(
          m_backLeftInputs.turnPositionRad, new Rotation2d(m_backLeftInputs.drivePositionMeters)),
      new SwerveModulePosition(
          m_backRightInputs.turnPositionRad, new Rotation2d(m_backRightInputs.drivePositionMeters)),
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
      new SwerveModuleState(
          m_frontLeftInputs.driveVelocityMetersPerSec,
          new Rotation2d(m_frontLeftInputs.turnPositionRad)),
      new SwerveModuleState(
          m_frontRightInputs.driveVelocityMetersPerSec,
          new Rotation2d(m_frontRightInputs.turnPositionRad)),
      new SwerveModuleState(
          m_backLeftInputs.driveVelocityMetersPerSec,
          new Rotation2d(m_backLeftInputs.turnPositionRad)),
      new SwerveModuleState(
          m_backRightInputs.driveVelocityMetersPerSec,
          new Rotation2d(m_backRightInputs.turnPositionRad)),
    };
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 4.125);

    desiredStates[0] =
        SwerveModuleState.optimize(
            desiredStates[0], new Rotation2d(m_frontLeftInputs.turnPositionRad));
    m_frontLeft.setDriveVoltage((desiredStates[0].speedMetersPerSecond / 4.125) * 12);
    m_frontLeft.setTurnVoltage(
        m_turnController[0].calculate(
                m_frontLeftInputs.turnPositionRad, desiredStates[0].angle.getRadians())
            * 6);

    desiredStates[1] =
        SwerveModuleState.optimize(
            desiredStates[1], new Rotation2d(m_frontRightInputs.turnPositionRad));
    m_frontRight.setDriveVoltage((desiredStates[1].speedMetersPerSecond / 4.125) * 12);
    m_frontRight.setTurnVoltage(
        m_turnController[1].calculate(
                m_frontRightInputs.turnPositionRad, desiredStates[1].angle.getRadians())
            * 6);

    desiredStates[2] =
        SwerveModuleState.optimize(
            desiredStates[2], new Rotation2d(m_backLeftInputs.turnPositionRad));
    m_backLeft.setDriveVoltage((desiredStates[2].speedMetersPerSecond / 4.125) * 12);
    m_backLeft.setTurnVoltage(
        m_turnController[2].calculate(
                m_backLeftInputs.turnPositionRad, desiredStates[2].angle.getRadians())
            * 6);

    desiredStates[3] =
        SwerveModuleState.optimize(
            desiredStates[3], new Rotation2d(m_backRightInputs.turnPositionRad));
    m_backRight.setDriveVoltage((desiredStates[3].speedMetersPerSecond / 4.125) * 12);
    m_backRight.setTurnVoltage(
        m_turnController[3].calculate(
                m_backRightInputs.turnPositionRad, desiredStates[3].angle.getRadians())
            * 6);
  }
}
