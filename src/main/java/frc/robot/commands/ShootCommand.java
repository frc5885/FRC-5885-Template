// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.io.Beambreak;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {

  private FeederSubsystem m_feederSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private Beambreak m_beambreak;

  /** Creates a new Shoot. */
  public ShootCommand(
      FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, Beambreak beambreak) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feederSubsystem = feederSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_beambreak = beambreak;

    addRequirements(m_feederSubsystem, m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterSubsystem.isVelocityTerminal()) {
      m_feederSubsystem.intake();
    } else {
      m_feederSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feederSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_beambreak.isOpen()) {
      return true;
    } else {
      return false;
    }
  }
}
