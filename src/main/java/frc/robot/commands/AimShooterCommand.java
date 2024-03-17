// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.io.DriverController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShooterCommand extends Command {

  ShooterSubsystem m_shooterSubsystem;
  DriverController m_driverController;

  /** Creates a new SpinShooterCMD. */
  public AimShooterCommand(
      DriverController driverController,
      ShooterSubsystem shooterSubsystem
  ) {
    m_shooterSubsystem = shooterSubsystem;
    m_driverController = driverController;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.spinFast();
    if (m_shooterSubsystem.isVelocityTerminal()) {
      m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
