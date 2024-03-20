// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.DriverController;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.base.subsystems.swerve.SwerveAction;
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

  /** Creates a new AutoAimShooterCommand. */
  public AutoAimShooterCommand(
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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceToTarget = m_photonVision.getDistanceToTarget(
        m_swervePoseEstimator.getPose(), m_photonVision.getTargetID());
    double correctionFactor = 1.0;
    double wristAngle = (0.07356 * Math.atan(2.00 / distanceToTarget) + 0.2927)
        * correctionFactor; // Jack Frias special
    // double wristAngle = SmartDashboard.getNumber("SHOOTPOINT",
    // Constants.kWristAmp);
    SmartDashboard.putNumber("DISTANCE", distanceToTarget);

    m_shooterSubsystem.spinFastClose();

    if (wristAngle >= Constants.kWristEncoderMin && wristAngle <= Constants.kWristStow) {
      m_wristSubsystem.pos(wristAngle);
    } else {
      m_wristSubsystem.pos(Constants.kWristStow);
    }
    if (m_shooterSubsystem.isVelocityTerminal()) {
      m_feederSubsystem.intake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wristSubsystem.pos(Constants.kWristStow);
    m_shooterSubsystem.stop();
    m_feederSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_beambreak.isOpen();
  }
}
