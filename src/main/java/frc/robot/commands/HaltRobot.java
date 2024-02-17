// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;

public class HaltRobot extends Command {

  SwerveDrive m_swerveDrive;
  ChassisSpeeds m_idleChassisSpeeds;

  /** Creates a new HaltRobot. */
  public HaltRobot(SwerveDrive swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_swerveDrive = swerveDrive;
    m_idleChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.setChassisSpeeds(m_idleChassisSpeeds);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.setChassisSpeeds(m_idleChassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    ChassisSpeeds fieldSpeeds = m_swerveDrive.getChassisSpeeds();
    return (Math.abs(fieldSpeeds.vxMetersPerSecond) > 0.001
        || Math.abs(fieldSpeeds.vyMetersPerSecond) > 0.001
        || Math.abs(fieldSpeeds.omegaRadiansPerSecond) > 0.001);
  }
}
