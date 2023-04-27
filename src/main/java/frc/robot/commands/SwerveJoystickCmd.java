// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import java.util.function.Supplier;

public class SwerveJoystickCmd extends CommandBase {

  private final SwerveDrive m_swerveSubsystem;
  private final Supplier<Double> m_xSpdFunction, m_ySpdFunction, m_turningSpdFunction;
  private final Supplier<Boolean> m_fieldOrientedFunctions;
  private final SlewRateLimiter m_xLimiter, m_yLimiter, m_turningLimiter;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(
      SwerveDrive swerveSubsystem,
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Boolean> fieldOrientedFunction) {
    m_swerveSubsystem = swerveSubsystem;
    m_xSpdFunction = xSpdFunction;
    m_ySpdFunction = ySpdFunction;
    m_turningSpdFunction = turningSpdFunction;
    m_fieldOrientedFunctions = fieldOrientedFunction;

    m_xLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    m_yLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    m_turningLimiter = new SlewRateLimiter(SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = m_xSpdFunction.get();
    double ySpeed = m_ySpdFunction.get();
    double turningSpeed = m_turningSpdFunction.get();

    xSpeed = Math.abs(xSpeed) > SwerveConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > SwerveConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > SwerveConstants.kDeadband ? turningSpeed : 0.0;

    xSpeed = m_xLimiter.calculate(xSpeed) * SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = m_yLimiter.calculate(xSpeed) * SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed =
        m_turningLimiter.calculate(turningSpeed)
            * SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds;
    if (m_fieldOrientedFunctions.get()) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, turningSpeed, m_swerveSubsystem.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }
    
    SwerveModuleState[] moduleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  
    m_swerveSubsystem.setModuleStates(moduleStates);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
