package frc.robot.base.io;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class WCXboxController extends CommandXboxController {

  protected final JoystickButton xButton =
      new JoystickButton(getHID(), XboxController.Button.kX.value);
  protected final JoystickButton yButton =
      new JoystickButton(getHID(), XboxController.Button.kY.value);
  protected final JoystickButton aButton =
      new JoystickButton(getHID(), XboxController.Button.kA.value);
  protected final JoystickButton bButton =
      new JoystickButton(getHID(), XboxController.Button.kB.value);
  protected final JoystickButton leftBumper =
      new JoystickButton(getHID(), XboxController.Button.kLeftBumper.value);
  protected final JoystickButton rightBumper =
      new JoystickButton(getHID(), XboxController.Button.kRightBumper.value);
  protected final JoystickButton leftStick =
      new JoystickButton(getHID(), XboxController.Button.kLeftStick.value);
  protected final JoystickButton rightStick =
      new JoystickButton(getHID(), XboxController.Button.kRightStick.value);
  protected final JoystickButton backButton =
      new JoystickButton(getHID(), XboxController.Button.kBack.value);
  protected final JoystickButton startButton =
      new JoystickButton(getHID(), XboxController.Button.kStart.value);

  public WCXboxController(int port) {
    super(port);
  }

  public JoystickButton getXButton() {
    return xButton;
  }

  public JoystickButton getYButton() {
    return yButton;
  }

  public JoystickButton getAButton() {
    return aButton;
  }

  public JoystickButton getBButton() {
    return bButton;
  }

  public JoystickButton getLeftBumper() {
    return leftBumper;
  }

  public JoystickButton getRightBumper() {
    return rightBumper;
  }

  public JoystickButton getLeftStick() {
    return leftStick;
  }

  public JoystickButton getRightStick() {
    return rightStick;
  }

  public JoystickButton getBackButton() {
    return backButton;
  }

  public JoystickButton getStartButton() {
    return startButton;
  }
}
