// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    xLimiter = new SlewRateLimiter(12);
    yLimiter = new SlewRateLimiter(12);
    turningLimiter = new SlewRateLimiter(8);
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

    // x * y --> y === speed constant, m/s
    xSpd = xLimiter.calculate(xSpd) * 1.5;
    ySpd = yLimiter.calculate(ySpd) * 1.5;
    turnSpd = turningLimiter.calculate(turnSpd) * 3.14159 * 2;

    ChassisSpeeds chassisSpeeds;
    // Use field oriented drive
    if (true) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpd, ySpd, turnSpd, m_swerveSubsystem.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpd, ySpd, turnSpd);
    }

    // Comment below out if problem occures
    chassisSpeeds =
        discretize(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond,
            0.02);

    SwerveModuleState[] moduleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    Logger.getInstance().recordOutput("Expected Chassis Speeds", moduleStates);
    // Logger.getInstance().recordOutput("Expected Chassis Speeds", m_swerveSubsystem.getHeading());

    m_swerveSubsystem.setModuleStates(moduleStates);
  }

  // 2024
  // https://github.wpilib.org/allwpilib/docs/development/java/edu/wpi/first/math/kinematics/ChassisSpeeds.html#discretize(edu.wpi.first.math.kinematics.ChassisSpeeds,double)
  // Right now
  // https://github.com/cachemoney8096/2023-charged-up/blob/main/src/main/java/frc/robot/subsystems/drive/DriveSubsystem.java#L174
  public static ChassisSpeeds discretize(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      double dtSeconds) {
    var desiredDeltaPose =
        new Pose2d(
            vxMetersPerSecond * dtSeconds,
            vyMetersPerSecond * dtSeconds,
            new Rotation2d(omegaRadiansPerSecond * dtSeconds));
    var twist = new Pose2d().log(desiredDeltaPose);
    return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
  }

  /**
   * Logical inverse of the above. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/b5da3c760b78d598b492e1cc51d8331c2ad50f6a/src/main/java/com/team254/lib/geometry/Pose2d.java
   */
  // public static Twist2d log(final Pose2d transform) {
  //   final double dtheta = transform.getRotation().getRadians();
  //   final double half_dtheta = 0.5 * dtheta;
  //   final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
  //   double halftheta_by_tan_of_halfdtheta;
  //   if (Math.abs(cos_minus_one) < 1E-9) {
  //     halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
  //   } else {
  //     halftheta_by_tan_of_halfdtheta =
  //         -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
  //   }
  //   final Translation2d translation_part =
  //       transform
  //           .getTranslation()
  //           .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
  //   return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
