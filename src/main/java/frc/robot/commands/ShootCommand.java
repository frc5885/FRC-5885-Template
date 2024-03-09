// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.FeederSubsystem;

public class ShootCommand extends Command {

  private FeederSubsystem m_feederSubsystem;

  private CommandXboxController m_xboxController;

  /** Creates a new Shoot. */
  public ShootCommand(FeederSubsystem feederSubsystem, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feederSubsystem = feederSubsystem;
    m_xboxController = xboxController;

    addRequirements(m_feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double position = m_xboxController.getRightTriggerAxis();
    SmartDashboard.putNumber("RightTrigeerPosition", position);
    if (position > 0.1) {
      m_feederSubsystem.intake();
    } else {
      m_feederSubsystem.stop();
    }
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
