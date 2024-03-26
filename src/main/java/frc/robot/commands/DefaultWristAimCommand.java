// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.WCLogger;
import frc.robot.Robot;
import frc.robot.WristAngleUtil;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.DriverController;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class DefaultWristAimCommand extends Command {

  DriverController m_driverController;
  WristSubsystem m_wristSubsystem;
  ArmSubsystem m_armSubsystem;
  Robot m_robot;
  PhotonVisionSystem m_photonVision;
  SwervePoseEstimator m_swervePoseEstimator;
  Beambreak m_beambreak;

  /** Creates a new SpinShooterCMD. */
  public DefaultWristAimCommand(
      DriverController driverController,
      Robot robot,
      WristSubsystem wristSubsystem,
      PhotonVisionSystem photonVision,
      SwervePoseEstimator swervePoseEstimator,
      Beambreak beambreak) {
    m_driverController = driverController;
    m_robot = robot;
    m_photonVision = photonVision;
    m_swervePoseEstimator = swervePoseEstimator;
    m_wristSubsystem = wristSubsystem;
    m_beambreak = beambreak;
    addRequirements(m_wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_beambreak.isBroken() && m_armSubsystem.isArmDown()) {
      double distanceToTarget =
          m_photonVision.getDistanceToTarget(
              m_swervePoseEstimator.getPose(), m_photonVision.getTargetID());
      double wristAngle = WristAngleUtil.getAngle(distanceToTarget);
      // double wristAngle = SmartDashboard.getNumber("SHOOTPOINT", Constants.kWristAmp);
      if (distanceToTarget < 3.1) {
        m_wristSubsystem.pos(wristAngle);
      } else if (!m_wristSubsystem.isStowed()){
        m_wristSubsystem.pos(Constants.kWristStow);
      } else {
        m_wristSubsystem.stop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_beambreak.isOpen() || !m_armSubsystem.isArmDown();
  }
}
