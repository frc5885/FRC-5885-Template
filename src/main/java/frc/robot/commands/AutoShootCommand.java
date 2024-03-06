// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootCommand extends SequentialCommandGroup {

  private final FeederSubsystem m_feederSubsystem;
  private final Robot m_robot;

  public AutoShootCommand(Robot robot, FeederSubsystem feederSubsystem) {

    m_feederSubsystem = feederSubsystem;
    m_robot = robot;

    addCommands(
      new InstantCommand(() -> m_robot.setAimBotting(true)),
      new WaitCommand(2),
      new InstantCommand(() -> m_feederSubsystem.intake()),
      new WaitCommand(2),
      new InstantCommand(() -> m_feederSubsystem.stop()),
      new InstantCommand(() -> m_robot.setAimBotting(false))
    );
    addRequirements(m_feederSubsystem);
  }
}
