package frc.robot.base.io;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DriverController extends WCXboxController {

  public DriverController(Command leftStickCommand, Command rightStickCommand) {
    super(ControllerConstants.kDriverControllerPort);
    leftStick.onTrue(leftStickCommand);
    rightStick.onTrue(rightStickCommand);
  }

  @Override
  public JoystickButton getLeftStick() {
    throw new RuntimeException("Left stick is reserved for resetting the swerve gyro!");
  }

  @Override
  public JoystickButton getRightStick() {
    throw new RuntimeException("Right stick is reserved for changing swerve field orientation!");
  }

  @Override
  public double getLeftY() {
    return -super.getLeftY();
  }

  @Override
  public double getRightY() {
    return -super.getRightY();
  }
}
