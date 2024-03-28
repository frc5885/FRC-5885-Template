package frc.robot.base.io;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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

  @Override
  public double getLeftY() {
    return -super.getLeftY();
  }

  @Override
  public double getRightY() {
    return -super.getRightY();
  }

  public void scheduleOnRightTrigger(Command command) {
    scheduleOnRightTrigger(command, 0.1);
  }

  public void scheduleOnRightTrigger(Command command, double minimumInput) {
    scheduleOnInput(command, () -> getRightTriggerAxis() > minimumInput);
  }

  public void scheduleOnLeftTriggerTrue(Command command) {
    scheduleOnLeftTriggerTrue(command, 0.1);
  }

  public void scheduleOnLeftTriggerFalse(Command command) {
    scheduleOnLeftTriggerFalse(command, 0.1);
  }

  public void scheduleOnLeftTriggerTrue(Command command, double minimumInput) {
    scheduleOnInput(command, () -> getLeftTriggerAxis() > minimumInput);
  }

  public void scheduleOnLeftTriggerFalse(Command command, double minimumInput) {
    scheduleOnInput(command, () -> getLeftTriggerAxis() <= minimumInput);
  }

  private void scheduleOnInput(Command command, BooleanSupplier condition) {
    EventLoop looper = CommandScheduler.getInstance().getDefaultButtonLoop();
    looper.bind(
        new Runnable() {
          private boolean wasTriggered = condition.getAsBoolean();

          @Override
          public void run() {
            boolean isTriggered = condition.getAsBoolean();
            if (!wasTriggered && isTriggered) {
              command.schedule();
            } else if (wasTriggered && !isTriggered) {
              command.cancel();
            }
            wasTriggered = isTriggered;
          }
        });
  }
}
