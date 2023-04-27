// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleNEO implements SwerveModuleIO {
  /** Creates a new SwerveModuleNEO. */
  private final CANSparkMax m_driveMotor;

  private final CANSparkMax m_turnMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final PIDController m_turningPIDController;

  private final AnalogInput m_absoluteEncoder;
  private final boolean m_absoluteEncoderReversed;
  private final double m_absoluteEncoderOffsetRad;

  public SwerveModuleNEO(
      int driveMotorID,
      int turningMotorID,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      int absoluteEncoderID,
      boolean absoluteEncoderReversed,
      double absoluteEncoderOffsetRad) {

    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_turnMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

    m_absoluteEncoder = new AnalogInput(absoluteEncoderID);
    m_absoluteEncoderReversed = absoluteEncoderReversed;
    m_absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

    m_driveEncoder = m_driveMotor.getEncoder(Type.kHallSensor, 42);
    m_turningEncoder = m_turnMotor.getEncoder(Type.kHallSensor, 42);

    m_driveEncoder.setPositionConversionFactor(SwerveConstants.Module.kDriveEncoderRot2Meter);
    m_driveEncoder.setVelocityConversionFactor(SwerveConstants.Module.kDriveEncoderRPM2MeterPerSec);
    m_turningEncoder.setPositionConversionFactor(SwerveConstants.Module.kTurningEncoderRot2Rad);
    m_turningEncoder.setPositionConversionFactor(
        SwerveConstants.Module.kTurningEncoderRPM2RadPerSec);

    m_turningPIDController = new PIDController(SwerveConstants.Module.kPTurning, 0, 0);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoder();
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.drivePositionMeters = getDrivePosition();
    inputs.driveVelocityMetersPerSec = getDriveVelocity();
    inputs.driveTemperature = m_driveMotor.getMotorTemperature();
    inputs.driveBusVoltage = m_driveMotor.getBusVoltage();
    inputs.driveCurrent = m_driveMotor.getOutputCurrent();

    inputs.turnPositionRad = getTurningPosition();
    inputs.turnVelocityRadPerSec = getTurningVelocity();
    inputs.turnTemperature = m_driveMotor.getMotorTemperature();
    inputs.turnBusVoltage = m_driveMotor.getBusVoltage();
    inputs.turnCurrent = m_driveMotor.getOutputCurrent();
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveMotor.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnMotor.setVoltage(volts);
  }

  public double getDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return m_turningEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return m_turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    double angle = m_absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= m_absoluteEncoderOffsetRad;
    return angle * (m_absoluteEncoderReversed ? -1 : 1);
  }

  public void resetEncoder() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    m_driveMotor.set(
        state.speedMetersPerSecond / SwerveConstants.Module.kPhysicalMaxSpeedMetersPerSecond);
    m_turnMotor.set(
        m_turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turnMotor.set(0);
  }
}
