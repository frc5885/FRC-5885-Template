// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

  WPI_TalonFX m_LeftFrontMotor = new WPI_TalonFX(Constants.DrivetrainConstants.kLeftFrontMotorPort);
  WPI_TalonFX m_LeftRearMotor = new WPI_TalonFX(Constants.DrivetrainConstants.kLeftRearMotorPort);
  WPI_TalonFX m_RightFrontMotor =
      new WPI_TalonFX(Constants.DrivetrainConstants.kRightFrontMotorPort);
  WPI_TalonFX m_RightRearMotor = new WPI_TalonFX(Constants.DrivetrainConstants.kRightRearMotorPort);

  MotorControllerGroup m_LeftMotors = new MotorControllerGroup(m_LeftFrontMotor, m_LeftRearMotor);
  MotorControllerGroup m_RightMotors =
      new MotorControllerGroup(m_RightFrontMotor, m_RightRearMotor);

  AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  Encoder m_LeftEncoder =
      new Encoder(
          Constants.DrivetrainConstants.kLeftEncoderPortA,
          Constants.DrivetrainConstants.kLeftEncoderPortB,
          Constants.DrivetrainConstants.kLeftEncoderReversed);
  Encoder m_RightEncoder =
      new Encoder(
          Constants.DrivetrainConstants.kRightEncoderPortA,
          Constants.DrivetrainConstants.kRightEncoderPortB,
          Constants.DrivetrainConstants.kRightEncoderReversed);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_LeftMotors, m_RightMotors);

  private final DifferentialDriveOdometry m_odometry;

  public DrivetrainSubsystem() {
    m_LeftMotors.setInverted(Constants.DrivetrainConstants.kLeftMotorsReversed);
    m_RightMotors.setInverted(Constants.DrivetrainConstants.kRightMotorsReversed);

    m_LeftEncoder.setDistancePerPulse(Constants.DrivetrainConstants.kEncoderDistancePerPulse);
    m_RightEncoder.setDistancePerPulse(Constants.DrivetrainConstants.kEncoderDistancePerPulse);

    resetEncoders();

    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), m_LeftEncoder.getDistance(), m_RightEncoder.getDistance());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(), m_LeftEncoder.getDistance(), m_RightEncoder.getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return new DifferentialDriveWheelSpeeds(m_LeftEncoder.getRate(), m_RightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose) {

    resetEncoders();

    m_odometry.resetPosition(
        m_gyro.getRotation2d(), m_LeftEncoder.getDistance(), m_RightEncoder.getDistance(), pose);
  }

  public void arcadeDrive(double fwd, double rot) {

    m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {

    m_LeftMotors.setVoltage(leftVolts);

    m_RightMotors.setVoltage(rightVolts);

    m_drive.feed();
  }

  public double getAverageEncoderDistance() {

    return (m_LeftEncoder.getDistance() + m_RightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder() {

    return m_LeftEncoder;
  }

  public Encoder getRightEncoder() {

    return m_RightEncoder;
  }

  public void setMaxOutput(double maxOutput) {

    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {

    m_gyro.reset();
  }

  public double getHeading() {

    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {

    return -m_gyro.getRate();
  }

  public void resetEncoders() {
    m_LeftEncoder.reset();
    m_RightEncoder.reset();
  }
}
