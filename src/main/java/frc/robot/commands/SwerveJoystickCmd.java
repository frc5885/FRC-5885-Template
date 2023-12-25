// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SwerveJoystickCmd extends CommandBase {

  private final SwerveDrive m_swerveSubsystem;
  private final SwervePoseEstimator m_poseEstimator;
  private final Supplier<Double> m_xDrivePercentFunction,
      m_yDrivePercentFunction,
      m_turnDrivePercentFunction;
  private final Supplier<Boolean> m_fieldOrientedFunction;

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
    double magnitude =
        MathUtil.applyDeadband(Math.hypot(xDir, yDir), ControllerConstants.kSwerveDriveDeadband);
    double linearVelocity = Math.pow(magnitude, 2) * SwerveConstants.kMaxSpeedXMetersPerSecond;
    Rotation2d linearDirection = new Rotation2d(xDir, yDir);

    double angularVelocity =
        MathUtil.applyDeadband(
                m_turnDrivePercentFunction.get(), ControllerConstants.kSwerveDriveDeadband)
            * SwerveConstants.kMaxSpeedAngularRadiansPerSecond;

    Translation2d translation = new Translation2d(linearVelocity, linearDirection);

    ChassisSpeeds chassisSpeeds;

    // Use field oriented drive
    if (m_fieldOrientedFunction.get()) {

      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(),
              translation.getY(),
              angularVelocity,
              m_poseEstimator
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

    Logger.getInstance().recordOutput("expected_module_states", moduleStates);
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
