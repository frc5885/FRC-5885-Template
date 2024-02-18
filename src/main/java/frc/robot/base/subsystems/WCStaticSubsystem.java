package frc.robot.base.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
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

  final protected void forwardMotors() {
    for (MotorController motor : motors) {
      motor.setVoltage(getBaseSpeed() * baseVoltage);
    }
  }

  final protected void reverseMotors() {
    for (MotorController motor : motors) {
      motor.setVoltage(-getBaseSpeed() * baseVoltage);
    }
  }

  final protected void stopMotors() {
    for (MotorController motor : motors) {
      motor.setVoltage(0);
    }
  }
}
