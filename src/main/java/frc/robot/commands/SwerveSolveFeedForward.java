// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import java.util.LinkedList;
import java.util.List;

public class SwerveSolveFeedForward extends CommandBase {

  private final SwerveDrive m_swerveSubsystem;

  private final Timer m_timer = new Timer();

  private final double m_delay = 3.0;
  private final double m_rampDelay = 0.3;
  private final double m_rampRate = 0.025;
  private final double m_minimumWaitTime = 0.5;

  private final List<Double> m_xVelocityMetersPerSecond = new LinkedList<Double>();
  private final List<Double> m_yVoltage = new LinkedList<Double>();

  private double m_currentVoltage = 0.0;
  private double m_previousVelocityMetersPerSecond = 0.0;
  private double m_lastSetTime = 0.0;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveSolveFeedForward(SwerveDrive swerveDrive) {
    m_swerveSubsystem = swerveDrive;

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Align wheels
    m_swerveSubsystem.setModulesAngle(0);
    double currentVelocityMetersPerSecond = m_swerveSubsystem.getChassisSpeeds().vxMetersPerSecond;

    if (m_timer.get() < m_delay) {
      return;
    } else {
      m_swerveSubsystem.setVoltage(m_currentVoltage);

      if ((currentVelocityMetersPerSecond - m_previousVelocityMetersPerSecond) / 0.02 <= 0.005
          && (m_timer.get() - m_lastSetTime) >= m_minimumWaitTime) {
        m_xVelocityMetersPerSecond.add(currentVelocityMetersPerSecond);
        m_yVoltage.add(m_currentVoltage);
        m_lastSetTime = m_timer.get();

        m_currentVoltage += m_rampRate;

        System.out.println("Steady state reached, increasing voltage to " + m_currentVoltage);
      }
    }

    m_previousVelocityMetersPerSecond = currentVelocityMetersPerSecond;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stop();

    for (int i = 0; i != m_xVelocityMetersPerSecond.size(); ++i) {
      System.out.println(m_xVelocityMetersPerSecond.get(i) + ", " + m_yVoltage.get(i));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}