// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmDownCmd extends Command {
  ArmSubsystem m_armSubsystem;
  WristSubsystem m_wristSubsystem;

  /** Creates a new ArmDownCmd. */
  public ArmDownCmd(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {
    m_armSubsystem = armSubsystem;
    m_wristSubsystem = wristSubsystem;
    addRequirements(m_armSubsystem, m_wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_armSubsystem.isArmDown()) {
      m_armSubsystem.pos(Constants.kArmStow);
    } else {
      m_armSubsystem.stop();
    }
    if (m_armSubsystem.isArmDown() && !m_wristSubsystem.isStowed()) {
      m_wristSubsystem.pos(Constants.kWristStow);
    } else {
      m_wristSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wristSubsystem.stop();
    m_armSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubsystem.isArmDown() && m_wristSubsystem.isStowed();
  }
}
