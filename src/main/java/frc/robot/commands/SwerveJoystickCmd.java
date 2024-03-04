// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.modules.swerve.SwerveConstants;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SwerveJoystickCmd extends Command {

  private final SwerveDriveSubsystem m_swerveSubsystem;
  private final SwervePoseEstimator m_poseEstimator;
  private final PhotonVisionSystem m_photonVision;
  private final Supplier<Double> m_xDrivePercentFunction,
      m_yDrivePercentFunction,
      m_turnDrivePercentFunction;
  private final Supplier<Boolean> m_fieldOrientedFunction;
  private final Supplier<Boolean> m_aimBotFunction;

  private PIDController m_aimBotPID;

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
      SwerveDriveSubsystem swerveSubsystem,
      SwervePoseEstimator poseEstimator,
      PhotonVisionSystem photonVision,
      Supplier<Double> xDrivePercentFunction,
      Supplier<Double> yDrivePercentFunction,
      Supplier<Double> turnDrivePercentFunction,
      Supplier<Boolean> fieldOrientedFunction,
      Supplier<Boolean> aimBotFunction) {
    m_swerveSubsystem = swerveSubsystem;
    m_poseEstimator = poseEstimator;
    m_photonVision = photonVision;
    m_xDrivePercentFunction = xDrivePercentFunction;
    m_yDrivePercentFunction = yDrivePercentFunction;
    m_turnDrivePercentFunction = turnDrivePercentFunction;
    m_fieldOrientedFunction = fieldOrientedFunction;
    m_aimBotFunction = aimBotFunction;

    // F = 1.54hz
    m_aimBotPID = new PIDController(2.5, 0.25, 0.08);
    m_aimBotPID.enableContinuousInput(-Math.PI, Math.PI);
    m_aimBotPID.setTolerance(SwerveConstants.Module.kAimbotTolerance);

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
    magnitude = MathUtil.applyDeadband(magnitude, SwerveConstants.kSwerveDriveDeadband);

    // Square the magnitude (0-1), this gives us better control at slow speeds and
    // lets us go fast when we want to. This is a per person preferance and might
    // need to be changed.
    double magnitudeSqrd = Math.pow(magnitude, 2);

    double linearVelocity =
        magnitudeSqrd * SwerveConstants.kMaxSpeedMetersPerSecond * m_linearSpeedLimitChooser.get();
    Rotation2d linearDirection = new Rotation2d(xDir, yDir);

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    // Rotation speed stuff
    double angularVelocity;
    if (m_aimBotFunction.get()) {

      if (m_aimBotPID.atSetpoint()) {
        angularVelocity = 0;
      } else {
      Pose2d robotPose = m_poseEstimator.getPose();
      double angleToTarget = m_photonVision.getAngleToTarget(robotPose, m_photonVision.getTargetID());
      angularVelocity =
          m_aimBotPID.calculate(robotPose.getRotation().getRadians(), angleToTarget);
      }
    } else {
      angularVelocity =
          MathUtil.applyDeadband(
              m_turnDrivePercentFunction.get(), SwerveConstants.kSwerveDriveDeadband);
    }
    angularVelocity *=
        SwerveConstants.kMaxSpeedAngularRadiansPerSecond * m_angularSpeedLimitChooser.get();

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
                          m_swerveSubsystem.getAngularVelocity() * SwerveConstants.kDriftFactor))
                  .plus(
                      new Rotation2d(
                          Units.degreesToRadians(alliance == Alliance.Blue ? 0.0 : 180.0))));

    } else {
      chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), angularVelocity);
    }

    // chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

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
