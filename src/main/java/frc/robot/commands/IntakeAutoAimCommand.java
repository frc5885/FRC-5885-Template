// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Robot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.subsystems.swerve.SwerveAction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAutoAimCommand extends ParallelCommandGroup {

  Robot m_robot;
  Beambreak m_beambreak;
  IntakeSubsystem m_intakeSubsystem;
  FeederSubsystem m_feederSubsystem;
  WristSubsystem m_wristSubsystem;
  ArmSubsystem m_armSubsystem;

  /** Creates a new IntakeAutoAimCommand. */
  public IntakeAutoAimCommand(
      Robot robot,
      Beambreak beambreak,
      IntakeSubsystem intakeSubsystem,
      FeederSubsystem feederSubsystem,
      WristSubsystem wristSubsystem,
      ArmSubsystem armSubsystem) {
    m_robot = robot;
    m_beambreak = beambreak;
    m_intakeSubsystem = intakeSubsystem;
    m_feederSubsystem = feederSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_armSubsystem = armSubsystem;

    addCommands(
        new SetSwerveActionCommand(m_robot, SwerveAction.AIMNOTE),
        new IntakeCommand(
            m_beambreak, m_intakeSubsystem, m_feederSubsystem, m_wristSubsystem, m_armSubsystem),
        new StartEndCommand(
            () -> m_robot.setFieldOriented(false), () -> m_robot.setFieldOriented(true)));
  }
}
