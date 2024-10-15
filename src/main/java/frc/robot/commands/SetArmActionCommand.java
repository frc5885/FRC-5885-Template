package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.base.subsystems.ArmAction;
import frc.robot.base.subsystems.swerve.SwerveAction;

public class SetArmActionCommand extends Command {

  Robot m_robot;
  ArmAction m_action;

  public SetArmActionCommand(Robot robot, ArmAction action) {
    m_robot = robot;
    m_action = action;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_robot.setArmAction(m_action);
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
