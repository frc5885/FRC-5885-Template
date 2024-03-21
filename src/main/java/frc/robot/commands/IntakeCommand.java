// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.io.Beambreak;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCMD. */
  Beambreak m_beambreak;

  IntakeSubsystem m_intakeSubsystem;
  FeederSubsystem m_feederSubsystem;
  WristSubsystem m_wristSubsystem;
  ArmSubsystem m_armSubsystem;

  public IntakeCommand(
      Beambreak beambreak,
      IntakeSubsystem intakeSubsystem,
      FeederSubsystem feederSubsystem,
      WristSubsystem wristSubsystem,
      ArmSubsystem armSubsystem
  ) {
    m_beambreak = beambreak;
    m_intakeSubsystem = intakeSubsystem;
    m_feederSubsystem = feederSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_armSubsystem = armSubsystem;

    addRequirements(m_intakeSubsystem, m_feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakeSubsystem.getSubsystemAction() != SubsystemAction.OUTTAKE) {
      m_intakeSubsystem.intake();
      m_feederSubsystem.intake();
    }
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
    return m_beambreak.isBroken() || !m_wristSubsystem.isStowed() || !m_armSubsystem.isArmDown();
  }
}
