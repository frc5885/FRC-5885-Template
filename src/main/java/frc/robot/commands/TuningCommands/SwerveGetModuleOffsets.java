// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TuningCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import java.util.ArrayList;

public class SwerveGetModuleOffsets extends Command {

  private final SwerveDrive m_swerveSubsystem;

  private final int m_bufferSize = 100;
  private ArrayList<Double> m_leftFrontValues = new ArrayList<>();
  private ArrayList<Double> m_rightFrontValues = new ArrayList<>();
  private ArrayList<Double> m_leftRearValues = new ArrayList<>();
  private ArrayList<Double> m_rightRearValues = new ArrayList<>();

  /** Creates a new SwerveJoystickCmd. */
  public SwerveGetModuleOffsets(SwerveDrive swerveDrive) {
    m_swerveSubsystem = swerveDrive;

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leftFrontValues = new ArrayList<>();
    m_rightFrontValues = new ArrayList<>();
    m_leftRearValues = new ArrayList<>();
    m_rightRearValues = new ArrayList<>();

    m_leftFrontValues.ensureCapacity(m_bufferSize + 1);
    m_rightFrontValues.ensureCapacity(m_bufferSize + 1);
    m_leftRearValues.ensureCapacity(m_bufferSize + 1);
    m_rightRearValues.ensureCapacity(m_bufferSize + 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_leftFrontValues.add(
        m_swerveSubsystem.getAbsoluteEncoderValue(0)
            + ModuleConstants.kLeftFrontModuleOffset.getRadians());
    m_rightFrontValues.add(
        m_swerveSubsystem.getAbsoluteEncoderValue(1)
            + ModuleConstants.kRightFrontModuleOffset.getRadians());
    m_leftRearValues.add(
        m_swerveSubsystem.getAbsoluteEncoderValue(2)
            + ModuleConstants.kLeftRearModuleOffset.getRadians());
    m_rightRearValues.add(
        m_swerveSubsystem.getAbsoluteEncoderValue(3)
            + ModuleConstants.kRightRearModuleOffset.getRadians());

    if (m_leftFrontValues.size() > m_bufferSize) {
      m_leftFrontValues.remove(0);
      m_rightFrontValues.remove(0);
      m_leftRearValues.remove(0);
      m_rightRearValues.remove(0);
    }

    double leftFrontAverage = 0;
    double rightFrontAverage = 0;
    double leftRearAverage = 0;
    double rightRearAverage = 0;

    for (int i = 0; i < m_leftFrontValues.size(); i++) {
      leftFrontAverage += m_leftFrontValues.get(i);
      rightFrontAverage += m_rightFrontValues.get(i);
      leftRearAverage += m_leftRearValues.get(i);
      rightRearAverage += m_rightRearValues.get(i);
    }

    leftFrontAverage /= m_leftFrontValues.size();
    rightFrontAverage /= m_rightFrontValues.size();
    leftRearAverage /= m_leftRearValues.size();
    rightRearAverage /= m_rightRearValues.size();

    leftFrontAverage = Units.radiansToDegrees(leftFrontAverage);
    rightFrontAverage = Units.radiansToDegrees(rightFrontAverage);
    leftRearAverage = Units.radiansToDegrees(leftRearAverage);
    rightRearAverage = Units.radiansToDegrees(rightRearAverage);

    System.out.printf(
        "FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f\n",
        leftFrontAverage, rightFrontAverage, leftRearAverage, rightRearAverage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Delete arrays
    m_leftFrontValues = null;
    m_rightFrontValues = null;
    m_leftRearValues = null;
    m_rightRearValues = null;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
