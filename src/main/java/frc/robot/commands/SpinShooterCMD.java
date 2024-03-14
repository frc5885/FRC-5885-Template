// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.io.DriverController;
import frc.robot.base.io.OperatorController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinShooterCMD extends Command {

  DriverController m_driverController;
  ShooterSubsystem m_shooterSubsystem;
  ArmSubsystem m_armSubsystem;

  /** Creates a new SpinShooterCMD. */
  public SpinShooterCMD(DriverController driverController, ShooterSubsystem shooterSubsystem,
      ArmSubsystem armSubsystem) {
    m_driverController = driverController;
    m_shooterSubsystem = shooterSubsystem;
    m_armSubsystem = armSubsystem;

    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driverController.getLeftTriggerAxis() > 0.1) {
      m_shooterSubsystem.spinFast();
    } else if (m_armSubsystem.isArmUp()) {
      m_shooterSubsystem.spinSlow();
    } else {
      m_shooterSubsystem.stop();
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
