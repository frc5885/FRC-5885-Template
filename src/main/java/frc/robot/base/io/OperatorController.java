package frc.robot.base.io;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class OperatorController extends WCXboxController {

  public OperatorController() {
    super(ControllerConstants.kOperatorControllerPort);
  }

  public double getLeftXWithDeadband() {
    return MathUtil.applyDeadband(super.getLeftX(), Constants.kOperatorLeftDeadzone);
  }

  public double getLeftYWithDeadband() {
    return MathUtil.applyDeadband(super.getLeftY(), Constants.kOperatorLeftDeadzone);
  }

  public double getRightXWithDeadband() {
    return MathUtil.applyDeadband(super.getRightX(), Constants.kOperatorRightDeadzone);
  }

  public double getRightYWithDeadband() {
    return MathUtil.applyDeadband(super.getRightY(), Constants.kOperatorRightDeadzone);
  }
}
