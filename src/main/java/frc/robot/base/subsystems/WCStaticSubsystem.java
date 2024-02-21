package frc.robot.base.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public abstract class WCStaticSubsystem extends SubsystemBase {

  private final double baseVoltage = 12.0;
  protected SubsystemAction subsystemAction;
  private final List<MotorController> motors;

  protected abstract double getBaseSpeed();

  protected abstract List<MotorController> initMotors();

  protected WCStaticSubsystem() {
    this.motors = initMotors();
  }

  public void stop() {
    subsystemAction = null;
  }

  protected final void forwardMotors() {
    for (MotorController motor : motors) {
      motor.setVoltage(getBaseSpeed() * baseVoltage);
    }
  }

  protected final void reverseMotors() {
    for (MotorController motor : motors) {
      motor.setVoltage(-getBaseSpeed() * baseVoltage);
    }
  }

  protected final void stopMotors() {
    for (MotorController motor : motors) {
      motor.setVoltage(0);
    }
  }
}
