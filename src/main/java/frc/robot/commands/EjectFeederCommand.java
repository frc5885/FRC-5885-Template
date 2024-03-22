// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class EjectFeederCommand extends Command {

  WristSubsystem m_wristSubsystem;
  FeederSubsystem m_feederSubsystem;
  ArmSubsystem m_armSubsystem;

  long dwellStart = 0L;
  int dwellDuration = 500;

  /** Creates a new SpinShooterCMD. */
  public EjectFeederCommand(
      WristSubsystem wristSubsystem, FeederSubsystem feederSubsystem, ArmSubsystem armSubsystem) {
    m_wristSubsystem = wristSubsystem;
    m_feederSubsystem = feederSubsystem;
    m_armSubsystem = armSubsystem;
    addRequirements(wristSubsystem, feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wristSubsystem.pos(Constants.kWristEncoderMin);
    double buffer = 0.04;
    if (m_wristSubsystem.getWristPosition() <= Constants.kWristEncoderMin + buffer) {
      m_feederSubsystem.eject();
      if (dwellStart == 0) {
        dwellStart = System.currentTimeMillis();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new StowWristCommand(m_armSubsystem, m_wristSubsystem).schedule();
    m_feederSubsystem.stop();
    dwellStart = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dwellStart != 0 && dwellStart + dwellDuration <= System.currentTimeMillis();
  }
}
