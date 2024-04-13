// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.OperatorController;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.base.subsystems.swerve.SwerveAction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AimSpeakerShotCommand extends Command {

  ShooterSubsystem m_shooterSubsystem;
  OperatorController m_operatorController;
  WristSubsystem m_wristSubsystem;
  ArmSubsystem m_armSubsystem;
  Robot m_robot;
  Beambreak m_beambreak;
  PhotonVisionSystem m_photonVision;
  SwervePoseEstimator m_swervePoseEstimator;
  FeederSubsystem m_feederSubsystem;

  /** Creates a new SpinShooterCMD. */
  public AimSpeakerShotCommand(
      OperatorController operatorController,
      ShooterSubsystem shooterSubsystem,
      Robot robot,
      WristSubsystem wristSubsystem,
      ArmSubsystem armSubsystem,
      PhotonVisionSystem photonVision,
      SwervePoseEstimator swervePoseEstimator,
      FeederSubsystem feederSubsystem,
      Beambreak beambreak) {
    m_shooterSubsystem = shooterSubsystem;
    m_operatorController = operatorController;
    m_robot = robot;
    m_photonVision = photonVision;
    m_swervePoseEstimator = swervePoseEstimator;
    m_wristSubsystem = wristSubsystem;
    m_beambreak = beambreak;
    m_armSubsystem = armSubsystem;
    m_feederSubsystem = feederSubsystem;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.spinFastClose();
    if (m_beambreak.isBroken() && m_armSubsystem.isArmDown()) {
      m_feederSubsystem.setPhotonDied(true);
      m_wristSubsystem.pos(Constants.kWristSubwoofer);
      if (m_shooterSubsystem.isVelocityTerminal()) {
        m_operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
      } else {
        m_operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robot.setSwerveAction(SwerveAction.DEFAULT);
    m_operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    if (m_armSubsystem.isArmDown()) {
      m_shooterSubsystem.stop();
      m_wristSubsystem.stop();
      if (interrupted) {
        new StowWristCommand(m_armSubsystem, m_wristSubsystem).schedule();
      }
    }
    m_feederSubsystem.setPhotonDied(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_beambreak.isOpen() || !m_armSubsystem.isArmDown();
  }
}
