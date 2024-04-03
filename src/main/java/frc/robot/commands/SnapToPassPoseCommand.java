// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.base.subsystems.swerve.SwerveAction;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SnapToPassPoseCommand extends ParallelCommandGroup {
  public SnapToPassPoseCommand(Robot robot) {
    super(
      new InstantCommand(() -> robot.setFieldOriented(true)),
      new InstantCommand(() -> robot.setSwerveAction(SwerveAction.PASS)));
  }
}