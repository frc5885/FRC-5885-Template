// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleSim implements SwerveModuleIO {
  /** Creates a new SwerveModuleNEO. */
  private final FlywheelSim m_driveMotor;

  private final FlywheelSim m_turnMotor;

  private final PIDController m_turningPIDController;

  private double turnRelativePositionRad = 0.0;
  private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public SwerveModuleSim(
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      int absoluteEncoderID,
      boolean absoluteEncoderReversed,
      double absoluteEncoderOffsetRad) {

    m_driveMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    m_turnMotor = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

    m_turningPIDController = new PIDController(SwerveConstants.Module.kPTurning, 0, 0);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  double prevVel = 0;
  double mmmmeterdrive = 0;
  double mmmmeterturn = 0;

  double vel_drive = 0;

  double vel_turn = 0;

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    m_driveMotor.update(0.02);
    m_turnMotor.update(0.02);

    double angleDiffRad = m_turnMotor.getAngularVelocityRadPerSec() * 0.02;
    turnRelativePositionRad += angleDiffRad;
    turnAbsolutePositionRad += angleDiffRad;
    while (turnAbsolutePositionRad < 0) {
      turnAbsolutePositionRad += 2.0 * Math.PI;
    }
    while (turnAbsolutePositionRad > 2.0 * Math.PI) {
      turnAbsolutePositionRad -= 2.0 * Math.PI;
    }

    inputs.drivePositionMeters =
        inputs.drivePositionMeters + (m_driveMotor.getAngularVelocityRadPerSec() * 0.02);
    mmmmeterdrive = inputs.drivePositionMeters;

    inputs.driveVelocityMetersPerSec = (inputs.drivePositionMeters - prevVel) / 0.02;
    prevVel = inputs.drivePositionMeters;
    inputs.driveTemperature = 0;
    inputs.driveBusVoltage = driveAppliedVolts;
    inputs.driveCurrent = 0;

    inputs.turnPositionRad = turnAbsolutePositionRad;
    mmmmeterturn = turnAbsolutePositionRad;
    inputs.turnVelocityRadPerSec = m_turnMotor.getAngularVelocityRadPerSec();
    inputs.turnTemperature = 0;
    inputs.turnBusVoltage = turnAppliedVolts;
    inputs.turnCurrent = 0;

    vel_drive = inputs.driveVelocityMetersPerSec;
    vel_turn = inputs.turnVelocityRadPerSec;
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_driveMotor.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_turnMotor.setInputVoltage(turnAppliedVolts);
  }

  public double getDrivePosition() {
    return mmmmeterdrive;
  }

  public double getTurningPosition() {
    return mmmmeterturn;
  }

  public double getDriveVelocity() {
    return vel_drive;
  }

  public double getTurningVelocity() {
    return vel_turn;
  }

  // public double getAbsoluteEncoderRad() {
  //   double angle = m_absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
  //   angle *= 2.0 * Math.PI;
  //   angle -= m_absoluteEncoderOffsetRad;
  //   return angle * (m_absoluteEncoderReversed ? -1 : 1);
  // }

  // public void resetEncoder() {
  //   m_driveEncoder.setPosition(0);
  //   m_turningEncoder.setPosition(getAbsoluteEncoderRad());
  // }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    m_driveMotor.setInputVoltage(
        state.speedMetersPerSecond / SwerveConstants.Module.kPhysicalMaxSpeedMetersPerSecond);
    m_turnMotor.setInputVoltage(
        m_turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
  }

  @Override
  public void stop() {
    m_driveMotor.setInputVoltage(0);
    m_turnMotor.setInputVoltage(0);
  }
}
