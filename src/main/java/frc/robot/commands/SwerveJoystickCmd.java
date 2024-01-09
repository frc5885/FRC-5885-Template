// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SwerveJoystickCmd extends Command {

  private final SwerveDrive m_swerveSubsystem;
  private final SwervePoseEstimator m_poseEstimator;
  private final Supplier<Double> m_xDrivePercentFunction,
      m_yDrivePercentFunction,
      m_turnDrivePercentFunction;
  private final Supplier<Boolean> m_fieldOrientedFunction;

  private static final LoggedDashboardChooser<Double> m_linearSpeedLimitChooser =
      new LoggedDashboardChooser<>("Linear Speed Limit");
  private static final LoggedDashboardChooser<Double> m_angularSpeedLimitChooser =
      new LoggedDashboardChooser<>("Angular Speed Limit");

  static {
    m_linearSpeedLimitChooser.addDefaultOption("100%", 1.0);
    m_linearSpeedLimitChooser.addOption("75%", 0.75);
    m_linearSpeedLimitChooser.addOption("50%", 0.5);
    m_linearSpeedLimitChooser.addOption("25%", 0.25);
    m_angularSpeedLimitChooser.addDefaultOption("100%", 1.0);
    m_angularSpeedLimitChooser.addOption("75%", 0.75);
    m_angularSpeedLimitChooser.addOption("50%", 0.5);
    m_angularSpeedLimitChooser.addOption("25%", 0.25);
  }

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(
      SwerveDrive swerveSubsystem,
      SwervePoseEstimator poseEstimator,
      Supplier<Double> xDrivePercentFunction,
      Supplier<Double> yDrivePercentFunction,
      Supplier<Double> turnDrivePercentFunction,
      Supplier<Boolean> fieldOrientedFunction) {
    m_swerveSubsystem = swerveSubsystem;
    m_poseEstimator = poseEstimator;
    m_xDrivePercentFunction = xDrivePercentFunction;
    m_yDrivePercentFunction = yDrivePercentFunction;
    m_turnDrivePercentFunction = turnDrivePercentFunction;
    m_fieldOrientedFunction = fieldOrientedFunction;

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xDir = m_xDrivePercentFunction.get();
    double yDir = m_yDrivePercentFunction.get();

    // Using the deadband here instead of the controllers
    // improves the control over the direction of the robot
    // since we no longer snap to the x/y axis.
    //
    // Another proble is that the magnitude of the x/y axis
    // is not always 1.0, so we need to cap it to fix that.
    // Ideally we would normalize the values but the max values
    // are not always the same on every controller and position.
    double magnitude = MathUtil.clamp(Math.hypot(xDir, yDir), -1.0, 1.0);
    magnitude = MathUtil.applyDeadband(magnitude, ControllerConstants.kSwerveDriveDeadband);

    // Square the magnitude (0-1), this gives us better control at slow speeds and
    // lets us go fast when we want to. This is a per person preferance and might
    // need to be changed.
    double magnitudeSqrd = Math.pow(magnitude, 2);

    double linearVelocity =
        magnitudeSqrd * SwerveConstants.kMaxSpeedMetersPerSecond * m_linearSpeedLimitChooser.get();
    Rotation2d linearDirection = new Rotation2d(xDir, yDir);

    // Rotation speed stuff
    double angularVelocity =
        MathUtil.applyDeadband(
                m_turnDrivePercentFunction.get(), ControllerConstants.kSwerveDriveDeadband)
            * SwerveConstants.kMaxSpeedAngularRadiansPerSecond
            * m_angularSpeedLimitChooser.get();

    // This does the trig for us and lets us get the x/y velocity
    Translation2d translation = new Translation2d(linearVelocity, linearDirection);
    ChassisSpeeds chassisSpeeds;

    // Use field oriented drive
    if (m_fieldOrientedFunction.get()) {

      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(),
              translation.getY(),
              angularVelocity,
              m_poseEstimator // This is used to compensate for skew when driving and turning.
                  // No idea how this works, but it does.
                  .getPose()
                  .getRotation()
                  .plus(
                      new Rotation2d(
                          m_swerveSubsystem.getAngularVelocity()
                              * SwerveConstants.kSpinCorrectionFactor)));
    } else {
      chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), angularVelocity);
    }

    SwerveModuleState[] moduleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    Logger.recordOutput("SwerveJoystickCmd/expectedModuleStates", moduleStates);
    Logger.recordOutput("SwerveJoystickCmd/expectedVelocity", linearVelocity);
    Logger.recordOutput("SwerveJoystickCmd/expectedAngularVelocity", angularVelocity);
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
