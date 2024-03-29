// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.WristAngleUtil;
import frc.robot.base.RobotSystem;
import frc.robot.base.io.Beambreak;
import frc.robot.base.modules.swerve.SwerveConstants;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.base.subsystems.swerve.SwerveAction;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutoAimShooterCommand extends Command {
  ShooterSubsystem m_shooterSubsystem;
  FeederSubsystem m_feederSubsystem;
  WristSubsystem m_wristSubsystem;
  PhotonVisionSystem m_photonVision;
  SwervePoseEstimator m_swervePoseEstimator;
  Beambreak m_beambreak;
  Robot m_robot;
  SwerveDriveSubsystem m_SwerveDriveSubsystem;
  private final PIDController m_aimBotPID;
  private boolean hadNote = false;
  long dwellStart = 0;

  /** Creates a new AutoAimShooterCommand. */
  public AutoAimShooterCommand(
      Robot robot,
      SwerveDriveSubsystem swerveDriveSubsystem,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      WristSubsystem wristSubsystem,
      PhotonVisionSystem photonVision,
      SwervePoseEstimator swervePoseEstimator,
      Beambreak beambreak) {
    m_shooterSubsystem = shooterSubsystem;
    m_feederSubsystem = feederSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_photonVision = photonVision;
    m_swervePoseEstimator = swervePoseEstimator;
    m_beambreak = beambreak;
    m_robot = robot;
    m_SwerveDriveSubsystem = swerveDriveSubsystem;
    m_aimBotPID =
        new PIDController(
            SwerveConstants.AimBotConstants.kAutoAimbotP,
            SwerveConstants.AimBotConstants.kAutoAimbotI,
            SwerveConstants.AimBotConstants.kAutoAimbotD);
    m_aimBotPID.enableContinuousInput(-Math.PI, Math.PI);
    m_aimBotPID.setTolerance(SwerveConstants.AimBotConstants.kAimbotTolerance);
    // addRequirements(m_SwerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_beambreak.isBroken()) {
      m_robot.setSwerveAction(SwerveAction.AIMBOTTING);
      hadNote = true;

      Pose2d robotPose = m_swervePoseEstimator.getPose();
      double angleToTarget =
          m_photonVision.getAngleToTarget(robotPose, m_photonVision.getTargetID());
      double angularVelocity =
          m_aimBotPID.calculate(robotPose.getRotation().getRadians(), angleToTarget);

      if (m_aimBotPID.atSetpoint()) {
        angularVelocity = 0;
      }

      // apply the angular velocity to the swerve drive (it should never translate the
      // robot, only
      // rotate it)
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, angularVelocity);
      SwerveModuleState[] moduleStates =
          SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      m_SwerveDriveSubsystem.setModuleStates(moduleStates);

      double distanceToTarget =
          m_photonVision.getDistanceToTarget(
              m_swervePoseEstimator.getPose(), m_photonVision.getTargetID());
      double wristAngle = WristAngleUtil.getAngle(distanceToTarget); // Jack Frias special
      // double wristAngle = SmartDashboard.getNumber("SHOOTPOINT",
      // Constants.kWristAmp);

      if (wristAngle >= Constants.kWristEncoderMin && wristAngle <= Constants.kWristStow) {
        m_wristSubsystem.pos(wristAngle);
      } else {
        m_wristSubsystem.pos(Constants.kWristStow);
      }

      double wristPos = m_wristSubsystem.getWristPosition();
      double buffer = 0.02;
      if (m_shooterSubsystem.isVelocityTerminal()
          && wristPos >= wristAngle - buffer
          && wristPos <= wristAngle + buffer
          && m_aimBotPID.atSetpoint()) {
        if (dwellStart == 0) {
          dwellStart = System.currentTimeMillis();
        } else if (System.currentTimeMillis() - dwellStart >= 250) {
          m_feederSubsystem.intake();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wristSubsystem.pos(Constants.kWristStow);
    m_feederSubsystem.stop();
    m_robot.setSwerveAction(SwerveAction.DEFAULT);
    // stop the robot
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    SwerveModuleState[] moduleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    m_SwerveDriveSubsystem.setModuleStates(moduleStates);
    hadNote = false;
    dwellStart = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotSystem.isReal()
        ? (m_beambreak.isOpen() && hadNote)
        : m_shooterSubsystem.isVelocityTerminal();
  }
}
