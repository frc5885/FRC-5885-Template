// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  private final ClimberSubsystem m_climberSubsystem;

  private final CommandXboxController m_xboxContoller;

  public ClimberCommand(ClimberSubsystem climberSubsystem, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_climberSubsystem = climberSubsystem;
    m_xboxContoller = xboxController;

    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPosition = m_xboxContoller.getLeftY();
    double rightPosition = m_xboxContoller.getRightY();
    m_climberSubsystem.climb(leftPosition, rightPosition, Constants.kClimberSpeedFactor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
