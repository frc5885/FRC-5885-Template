// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.io.Beambreak;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends Command {
  private FeederSubsystem m_feederSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private Beambreak m_beambreak;

  /** Creates a new AutoIntakeCommand. */
  public AutoIntakeCommand(
      IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, Beambreak beambreak) {
    m_intakeSubsystem = intakeSubsystem;
    m_feederSubsystem = feederSubsystem;
    m_beambreak = beambreak;
    addRequirements(m_intakeSubsystem, feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.intake();
    m_feederSubsystem.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stop();
    m_feederSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_beambreak.isBroken();
  }
}
