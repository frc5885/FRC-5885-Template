// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import java.util.function.Supplier;

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
    turningLimiter = new SlewRateLimiter(6);
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

    xSpd = Math.abs(xSpd) > 0.05 ? xSpd : 0;
    ySpd = Math.abs(ySpd) > 0.05 ? ySpd : 0;
    turnSpd = Math.abs(turnSpd) > 0.05 ? turnSpd : 0;

    xSpd = xLimiter.calculate(xSpd);
    ySpd = yLimiter.calculate(ySpd);
    turnSpd = turningLimiter.calculate(turnSpd);

    Translation2d vel = m_swerveSubsystem.getFieldVelocity().getTranslation();
    Translation2d heading = new Translation2d(xSpd, ySpd);

    Translation2d err = vel.div(vel.getNorm()).minus(heading.div(heading.getNorm()));

    xSpd += -(!Double.isNaN(err.getX()) ? err.getX() : 0) * 0.95;
    ySpd += -(!Double.isNaN(err.getY()) ? err.getY() : 0) * 0.95;

    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpd * 4.2 / 1,
            ySpd * 4.2 / 1,
            turnSpd * 9.83236752175 / 2,
            m_swerveSubsystem.getRotation2d());

    SwerveModuleState[] moduleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // ChassisSpeeds updated = new ChassisSpeeds(robot_one_step.getX() );

    // Logger.getInstance().recordOutput("chassisSpeeds", chassisSpeeds);
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
