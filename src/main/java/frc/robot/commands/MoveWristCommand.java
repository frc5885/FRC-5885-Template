// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.subsystems.WristSubsystem;

public class MoveWristCommand extends Command {
  private WristSubsystem m_WristSubsystem;
  private SwervePoseEstimator m_PoseEstimator;
  private PhotonVisionSystem m_PhotonVisionSystem;
  private PIDController m_PidController;

  /** Creates a new MoveWristCommand. */
  public MoveWristCommand(
      WristSubsystem wristSubsystem,
      SwervePoseEstimator poseEstimator,
      PhotonVisionSystem photonVisionSystem) {
    m_WristSubsystem = wristSubsystem;
    m_PoseEstimator = poseEstimator;
    m_PhotonVisionSystem = photonVisionSystem;
    m_PidController = new PIDController(1, 0, 0);
    addRequirements(m_WristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
