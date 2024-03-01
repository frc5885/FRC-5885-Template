package frc.robot.base.io;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DriverController extends WCXboxController {

  public DriverController(
      Command leftStickCommand, Command rightStickCommand, Command leftBumperCommand) {
    super(ControllerConstants.kDriverControllerPort);
    leftStick.onTrue(leftStickCommand);
    rightStick.onTrue(rightStickCommand);
    leftBumper.onTrue(leftBumperCommand);
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
  public JoystickButton getLeftBumper() {
    throw new RuntimeException("Left bumper is reserved for aimbot!");
  }
}
