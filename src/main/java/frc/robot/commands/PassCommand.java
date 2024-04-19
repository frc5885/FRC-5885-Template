// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.subsystems.swerve.SwerveAction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class PassCommand extends Command {

  Robot m_robot;
  WristSubsystem m_wristSubsystem;
  FeederSubsystem m_feederSubsystem;
  ArmSubsystem m_armSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  Beambreak m_beambreak;
  long dwellStart = 0L;

  /** Creates a new PassCommand. */
  public PassCommand(
      Robot robot,
      WristSubsystem wristSubsystem,
      FeederSubsystem feederSubsystem,
      ArmSubsystem armSubsystem,
      ShooterSubsystem shooterSubsystem,
      Beambreak beambreak) {
    m_robot = robot;
    m_armSubsystem = armSubsystem;
    m_feederSubsystem = feederSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_beambreak = beambreak;

    addRequirements(m_armSubsystem, m_wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robot.setSwerveAction(SwerveAction.PASS);
    if (m_beambreak.isBroken()) {
      m_armSubsystem.pos(Constants.kArmPass);
      m_wristSubsystem.pos(Constants.kWristEncoderMin);
      m_shooterSubsystem.spinPass();
    }

    double wristPos = m_wristSubsystem.getWristPosition();
    double buffer = 0.02;
    if (wristPos >= Constants.kWristEncoderMin - buffer
        && wristPos <= Constants.kWristEncoderMin + buffer
        && m_robot.swerveIsAtSetpoint()) {
      if (dwellStart == 0) {
        dwellStart = System.currentTimeMillis();
      } else if (System.currentTimeMillis() - dwellStart >= 250) {
        m_feederSubsystem.shoot();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feederSubsystem.stop();
    m_shooterSubsystem.stop();
    if (!m_armSubsystem.isArmDown() && !interrupted) {
      new ArmDownCommand(m_armSubsystem, m_wristSubsystem).schedule();
    }
    m_robot.setSwerveAction(SwerveAction.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_beambreak.isOpen();
  }
}
