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
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AimShooterCommand extends Command {

  ShooterSubsystem m_shooterSubsystem;
  DriverController m_driverController;
  WristSubsystem m_WristSubsystem;
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
      PhotonVisionSystem photonVision,
      SwervePoseEstimator swervePoseEstimator,
      Beambreak beambreak) {
    m_shooterSubsystem = shooterSubsystem;
    m_driverController = driverController;
    m_robot = robot;
    m_photonVision = photonVision;
    m_swervePoseEstimator = swervePoseEstimator;
    m_WristSubsystem = wristSubsystem;
    m_beambreak = beambreak;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_beambreak.isBroken()) {
      m_robot.setSwerveAction(SwerveAction.AIMBOTTING);
      double distanceToTarget =
          m_photonVision.getDistanceToTarget(
              m_swervePoseEstimator.getPose(), m_photonVision.getTargetID());
      double correctionFactor = 1.0;
      double wristAngle =
          (0.07356 * Math.atan(2.00 / distanceToTarget) + 0.2927)
              * correctionFactor; // Jack Frias special
      // double wristAngle = SmartDashboard.getNumber("SHOOTPOINT", Constants.kWristAmp);
      SmartDashboard.putNumber("DISTANCE", distanceToTarget);

      // if (distanceToTarget >= 2.0) {
      //   m_shooterSubsystem.spinFastFar();
      // } else {
      //   m_shooterSubsystem.spinFastClose();
      // }
      m_shooterSubsystem.spinFast();

      if (wristAngle >= Constants.kWristEncoderMin && wristAngle <= Constants.kWristStow) {
        m_WristSubsystem.pos(wristAngle);
      } else {
        m_WristSubsystem.pos(Constants.kWristStow);
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
    m_shooterSubsystem.stop();
    m_WristSubsystem.stop();
    m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
