// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.WristAngleUtil;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.DriverController;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.base.subsystems.swerve.SwerveAction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AimShooterCommand extends Command {

  ShooterSubsystem m_shooterSubsystem;
  DriverController m_driverController;
  WristSubsystem m_wristSubsystem;
  ArmSubsystem m_armSubsystem;
  Robot m_robot;
  PhotonVisionSystem m_photonVision;
  SwervePoseEstimator m_swervePoseEstimator;
  Beambreak m_beambreak;

  /** Creates a new SpinShooterCMD. */
  public AimShooterCommand(
      DriverController driverController,
      ShooterSubsystem shooterSubsystem,
      Robot robot,
      WristSubsystem wristSubsystem,
      ArmSubsystem armSubsystem,
      PhotonVisionSystem photonVision,
      SwervePoseEstimator swervePoseEstimator,
      Beambreak beambreak) {
    m_shooterSubsystem = shooterSubsystem;
    m_driverController = driverController;
    m_robot = robot;
    m_photonVision = photonVision;
    m_swervePoseEstimator = swervePoseEstimator;
    m_wristSubsystem = wristSubsystem;
    m_beambreak = beambreak;
    m_armSubsystem = armSubsystem;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_beambreak.isBroken() && m_armSubsystem.isArmDown()) {
      m_robot.setSwerveAction(SwerveAction.AIMBOTTING);
      double distanceToTarget =
          m_photonVision.getDistanceToTarget(
              m_swervePoseEstimator.getPose(), m_photonVision.getTargetID());
      double wristAngle = WristAngleUtil.getAngle(distanceToTarget);
      // SmartDashboard.putNumber("DistanceToTarget", distanceToTarget);
      // double wristAngle = SmartDashboard.getNumber("SHOOTPOINT", Constants.kWristAmp);
      if (distanceToTarget >= 3.1) {
        m_shooterSubsystem.spinFastFar();
      } else {
        m_shooterSubsystem.spinFastClose();
      }

      if (distanceToTarget >= 5.0) {
        // Passing
        m_wristSubsystem.pos(Constants.kWristPass);
      } else if (wristAngle >= Constants.kWristEncoderMin && wristAngle <= Constants.kWristStow) {
        m_wristSubsystem.pos(wristAngle);
      } else {
        m_wristSubsystem.pos(Constants.kWristPass);
      }
      if (m_shooterSubsystem.isVelocityTerminal()) {
        m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
      } else {
        m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robot.setSwerveAction(SwerveAction.DEFAULT);
    m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    if (m_armSubsystem.isArmDown()) {
      m_shooterSubsystem.stop();
      m_wristSubsystem.stop();
      if (interrupted) {
        new StowWristCommand(m_armSubsystem, m_wristSubsystem).schedule();
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_beambreak.isOpen() || !m_armSubsystem.isArmDown();
  }
}
