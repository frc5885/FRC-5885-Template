// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.base.io.Beambreak;
import frc.robot.subsystems.*;

public class ArmUpCommand extends Command {
  ArmSubsystem m_armSubsystem;
  WristSubsystem m_wristSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  FeederSubsystem m_feederSubsystem;
  Beambreak m_beambreak;

  public ArmUpCommand(
      ArmSubsystem armSubsystem,
      WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      Beambreak beambreak
  ) {
    m_armSubsystem = armSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_beambreak = beambreak;
    m_feederSubsystem = feederSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.pos(Constants.kArmAmp);
    m_wristSubsystem.pos(Constants.kWristAmp);
    m_shooterSubsystem.spinSlow();
    if (m_beambreak.isOpen()) {
      m_feederSubsystem.intake();
    } else {
      m_feederSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_feederSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
