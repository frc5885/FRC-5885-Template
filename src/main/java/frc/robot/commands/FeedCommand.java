// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.io.Beambreak;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class FeedCommand extends Command {

  private FeederSubsystem m_feederSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private ArmSubsystem m_armSubsystem;
  private WristSubsystem m_wristSubsystem;
  private Beambreak m_beambreak;

  /** Creates a new Shoot. */
  public FeedCommand(
      FeederSubsystem feederSubsystem,
      ShooterSubsystem shooterSubsystem,
      ArmSubsystem armSubsystem,
      WristSubsystem wristSubsystem,
      Beambreak beambreak) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feederSubsystem = feederSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_beambreak = beambreak;
    m_armSubsystem = armSubsystem;
    m_wristSubsystem = wristSubsystem;

    addRequirements(m_feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterSubsystem.isVelocityTerminal() || m_armSubsystem.isArmUp()) {
      m_feederSubsystem.shoot();
    } else {
      m_feederSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feederSubsystem.stop();
    if (m_armSubsystem.isArmUp() && !interrupted) {
      new ArmDownCommand(m_armSubsystem, m_wristSubsystem).schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_beambreak.isOpen();
  }
}
