package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.base.subsystems.swerve.SwerveAction;

public class SetSwerveActionCommand extends Command {

  Robot m_robot;
  SwerveAction m_action;

  public SetSwerveActionCommand(Robot robot, SwerveAction action) {
    m_robot = robot;
    m_action = action;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_robot.setSwerveAction(m_action);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_robot.setSwerveAction(SwerveAction.DEFAULT);
  }
}
