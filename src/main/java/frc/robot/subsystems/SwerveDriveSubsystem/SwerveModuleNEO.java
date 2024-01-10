// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.SparkMaxConfigurer;

/** Add your docs here. */
public class SwerveModuleNEO implements SwerveModuleIO {

  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turnMotor;

  private AnalogInput m_turnAbsoluteEncoder;
  private Rotation2d m_turnAbsoluteEncoderOffset;

  private final RelativeEncoder m_driveDefaultEncoder;
  private final RelativeEncoder m_turnRelativeEncoder;

  public SwerveModuleNEO(
      int driveMotorId,
      int turnMotorId,
      int turnAbsoluteEncoderId,
      Rotation2d turnAbsoluteEncoderOffset,
      boolean turnMotorReversed,
      boolean driveMotorReversed) {
    m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    m_turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
    m_turnAbsoluteEncoder = new AnalogInput(turnAbsoluteEncoderId);
    m_turnAbsoluteEncoderOffset = turnAbsoluteEncoderOffset;

    m_driveDefaultEncoder = m_driveMotor.getEncoder();
    m_turnRelativeEncoder = m_turnMotor.getEncoder();

    m_driveMotor.setInverted(driveMotorReversed);
    m_turnMotor.setInverted(turnMotorReversed);

    // Optimize the spark max frame timings to clean up the CAN bus
    SparkMaxConfigurer.setFrameTimingsOptmized(m_driveMotor);
    SparkMaxConfigurer.setFrameTimingsOptmized(m_turnMotor);

    // This sets the update rate of the spark max position value
    // TODO: Motors has timeout issues, needs to find a way to detect and fix
    // m_driveMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 10);
    // if (!SwerveConstants.kUseExternalEncoders)
    //   m_turnMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 10);

    // This sets the conversion factor in the spark max, apparently
    // this can cause some issues. Needs investigating.
    m_driveDefaultEncoder.setPositionConversionFactor(
        SwerveConstants.ModuleConstants.kDriveEncoderRot2Meter);
    m_driveDefaultEncoder.setVelocityConversionFactor(
        SwerveConstants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    m_turnRelativeEncoder.setPositionConversionFactor(
        SwerveConstants.ModuleConstants.kTurningEncoderRot2Rad);
    m_turnRelativeEncoder.setVelocityConversionFactor(
        SwerveConstants.ModuleConstants.kTurningEncoderRPM2RadPerSec);

    m_turnRelativeEncoder.setPosition(getAbsoluteEncoderValue().getRadians());
  }

  public void updateInputs(SwerveModuleIOInputs inputs) {

    inputs.drivePositionMeters = m_driveDefaultEncoder.getPosition();
    inputs.driveVelocityMetersPerSec = m_driveDefaultEncoder.getVelocity();

    inputs.driveTemperatureCelsius = m_driveMotor.getMotorTemperature();
    inputs.driveCurrent = m_driveMotor.getOutputCurrent();
    inputs.driveVoltage = m_driveMotor.getAppliedOutput() * RobotController.getBatteryVoltage();

    inputs.turnAbsolutePositionRad = getAbsoluteEncoderValue().getRadians();

    // Use the absolute encoder value if UseExternalEncoders is true
    if (SwerveConstants.kUseExternalEncoders) {
      inputs.turnPositionRad = inputs.turnAbsolutePositionRad;
    } else {
      inputs.turnPositionRad = m_turnRelativeEncoder.getPosition();
    }

    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_turnRelativeEncoder.getVelocity())
            / (1 / SwerveConstants.ModuleConstants.kTurningMotorGearRatio);

    inputs.turnTemperature = m_turnMotor.getMotorTemperature();
    inputs.turnCurrent = m_turnMotor.getOutputCurrent();
    inputs.turnVoltage = m_turnMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
  }

  ////////////////////////////////////////
  // Calculate angle from absolute encoder
  public Rotation2d getAbsoluteEncoderValue() {
    double absolutePositionPercent =
        (m_turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V());
    return new Rotation2d(absolutePositionPercent * 2.0 * Math.PI)
        .minus(m_turnAbsoluteEncoderOffset);
  }

  public void setDriveVoltage(double voltage) {
    m_driveMotor.setVoltage(voltage);
  }

  public void setTurnVoltage(double voltage) {
    m_turnMotor.setVoltage(voltage);
  }

  public void setDriveBrakeMode(boolean enable) {
    m_driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setTurnBrakeMode(boolean enable) {
    m_turnMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
