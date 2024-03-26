package frc.robot.base.io;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class OperatorController extends WCXboxController {

  public OperatorController() {
    super(ControllerConstants.kOperatorControllerPort);
  }

  @Override
  public double getLeftY() {
    return -MathUtil.applyDeadband(super.getLeftY(), Constants.kOperatorLeftDeadzone);
  }

  @Override
  public double getLeftX() {
    return MathUtil.applyDeadband(super.getLeftX(), Constants.kOperatorLeftDeadzone);
  }

  @Override
  public double getRightX() {
    return MathUtil.applyDeadband(super.getRightX(), Constants.kOperatorRightDeadzone);
  }

  @Override
  public double getRightY() {
    return -MathUtil.applyDeadband(super.getRightY(), Constants.kOperatorRightDeadzone);
  }
}
