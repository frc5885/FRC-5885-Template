// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.DriverController;
import frc.robot.base.io.OperatorController;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCMD extends Command {
  /** Creates a new IntakeCMD. */
  Beambreak m_beambreak;
  IntakeSubsystem m_intakeSubsystem;
  FeederSubsystem m_feederSubsystem;
  DriverController m_driverController;

  public IntakeCMD(Beambreak beambreak, IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem,
      DriverController driverController) {
    m_beambreak = beambreak;
    m_intakeSubsystem = intakeSubsystem;
    m_feederSubsystem = feederSubsystem;
    m_driverController = driverController;

    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean rightBumperPressed = m_driverController.getRightBumper().getAsBoolean();
    if (rightBumperPressed && m_beambreak.isOpen()) {
      m_intakeSubsystem.intake();
      m_feederSubsystem.intake();
    } else {
      m_intakeSubsystem.stop();
      if (m_driverController.getRightTriggerAxis() < 0.1) {
        m_feederSubsystem.stop();
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
    return false;
  }
}
