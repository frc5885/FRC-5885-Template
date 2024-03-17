// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.base.io.OperatorController;
import frc.robot.subsystems.*;

public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  private final ClimberSubsystem m_climberSubsystem;

  private final OperatorController m_operatorController;

  public ClimberCommand(ClimberSubsystem climberSubsystem, OperatorController operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_climberSubsystem = climberSubsystem;
    m_operatorController = operatorController;

    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPosition = m_operatorController.getLeftYWithDeadband();
    double rightPosition = m_operatorController.getRightYWithDeadband();
    m_climberSubsystem.rightStickPosition(rightPosition);
    m_climberSubsystem.setLeftClimberSpeed(leftPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.kbru
  @Override
  public boolean isFinished() {
    return false;
  }
}
