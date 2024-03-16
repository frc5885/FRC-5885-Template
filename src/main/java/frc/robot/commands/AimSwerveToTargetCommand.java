// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.WCRobot;
import frc.robot.base.modules.swerve.SwerveConstants;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.base.subsystems.swerve.SwerveAction;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;

public class AimSwerveToTargetCommand extends Command {

  private final WCRobot m_robot;
  private final SwerveDriveSubsystem m_swerveDrive;
  private final SwervePoseEstimator m_poseEstimator;
  private final PhotonVisionSystem m_photonVision;
  private final PIDController m_aimBotPID;

  /** Creates a new AimSwerveToTargetCommand. */
  public AimSwerveToTargetCommand(
      WCRobot robot,
      SwerveDriveSubsystem swerveDrive,
      SwervePoseEstimator poseEstimator,
      PhotonVisionSystem photonVision) {
    m_robot = robot;
    m_swerveDrive = swerveDrive;
    m_poseEstimator = poseEstimator;
    m_photonVision = photonVision;
    m_aimBotPID = new PIDController(
        SwerveConstants.AimBotConstants.kAimbotP,
        SwerveConstants.AimBotConstants.kAimbotI,
        SwerveConstants.AimBotConstants.kAimbotD);
    m_aimBotPID.enableContinuousInput(-Math.PI, Math.PI);
    m_aimBotPID.setTolerance(SwerveConstants.AimBotConstants.kAimbotTolerance);

    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // same as aimbot from SwerveJoystickCmd
    Pose2d robotPose = m_poseEstimator.getPose();
    double angleToTarget = m_photonVision.getAngleToTarget(robotPose, m_photonVision.getTargetID());
    double angularVelocity = m_aimBotPID.calculate(robotPose.getRotation().getRadians(), angleToTarget);

    if (m_aimBotPID.atSetpoint()) {
      angularVelocity = 0;
    }

    // apply the angular velocity to the swerve drive (it should never translate the
    // robot, only
    // rotate it)
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, angularVelocity);
    SwerveModuleState[] moduleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    m_swerveDrive.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop the robot
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    SwerveModuleState[] moduleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    m_swerveDrive.setModuleStates(moduleStates);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
