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
import org.littletonrobotics.junction.Logger;

public class SwerveJoystickCmd extends CommandBase {

  private final SwerveDrive m_swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(
      SwerveDrive swerveDrive,
      Supplier<Double> xSpdFnc,
      Supplier<Double> ySpdFnc,
      Supplier<Double> turningSpd) {
    m_swerveSubsystem = swerveDrive;
    xSpdFunction = xSpdFnc;
    ySpdFunction = ySpdFnc;
    turningSpdFunction = turningSpd;
    xLimiter = new SlewRateLimiter(3);
    yLimiter = new SlewRateLimiter(3);
    turningLimiter = new SlewRateLimiter(3);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpd = xSpdFunction.get();
    double ySpd = ySpdFunction.get();
    double turnSpd = turningSpdFunction.get();

    xSpd = Math.abs(xSpd) > 0.05 ? xSpd : 0.0;
    ySpd = Math.abs(ySpd) > 0.05 ? ySpd : 0.0;
    turnSpd = Math.abs(turnSpd) > 0.05 ? turnSpd : 0.0;

    // x * y --> y === speed constant, m/s
    xSpd = xLimiter.calculate(xSpd) * 1.25;
    ySpd = yLimiter.calculate(ySpd) * 1.25;
    turnSpd = turningLimiter.calculate(turnSpd) * 3.14159;

    // TODO: Check 3rd order problem solution involving the tracking of the twist over time

    // xSpd = 0;
    // ySpd = 0;
    // turnSpd = 0;

    ChassisSpeeds chassisSpeeds;
    // Use field oriented drive
    if (false) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpd, ySpd, turnSpd, m_swerveSubsystem.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpd, ySpd, turnSpd);
    }

    SwerveModuleState[] moduleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // ChassisSpeeds updated = new ChassisSpeeds(robot_one_step.getX() );

    Logger.getInstance().recordOutput("chassisSpeedsvy", moduleStates);
    m_swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
