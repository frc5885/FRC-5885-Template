// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.base.RobotSystem;
import frc.robot.base.io.Beambreak;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutoSubwooferShotCommand extends Command {
  ShooterSubsystem m_shooterSubsystem;
  FeederSubsystem m_feederSubsystem;
  WristSubsystem m_wristSubsystem;
  Beambreak m_beambreak;
  long dwellStart = 0;
  private boolean hadNote = false;

  /** Creates a new AutoSubwooferShotCommand. */
  public AutoSubwooferShotCommand(
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      WristSubsystem wristSubsystem,
      Beambreak beambreak) {
    m_shooterSubsystem = shooterSubsystem;
    m_feederSubsystem = feederSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_beambreak = beambreak;
    // addRequirements(m_SwerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_beambreak.isBroken()) {
      hadNote = true;
    }
    // apply the angular velocity to the swerve drive (it should never translate the
    // robot, only
    // rotate it)
    double wristAngle = Constants.kWristSubwoofer;
    // double wristAngle = SmartDashboard.getNumber("SHOOTPOINT",
    // Constants.kWristAmp);

    if (wristAngle >= Constants.kWristEncoderMin && wristAngle <= Constants.kWristStow) {
      m_wristSubsystem.pos(wristAngle);
    } else {
      m_wristSubsystem.pos(Constants.kWristStow);
    }

    double wristPos = m_wristSubsystem.getWristPosition();
    double buffer = 0.02;
    if (/*m_shooterSubsystem.isVelocityTerminal()
        && /**/wristPos >= wristAngle - buffer
        && wristPos <= wristAngle + buffer) {
      if (dwellStart == 0) {
        dwellStart = System.currentTimeMillis();
      } else if (System.currentTimeMillis() - dwellStart >= 450) {
        m_feederSubsystem.shoot();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wristSubsystem.pos(Constants.kWristStow);
    m_feederSubsystem.stop();
    dwellStart = 0;
    hadNote = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotSystem.isReal()
        ? (m_beambreak.isOpen() && hadNote)
        : m_shooterSubsystem.isVelocityTerminal();
  }
}
