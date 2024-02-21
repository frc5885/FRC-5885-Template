package frc.robot.base.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class WCDualSubsystem extends SubsystemBase {

  private final double baseVoltage = 12.0;
  protected double speed1 = 0.0;
  protected double speed2 = 0.0;
  private final Pair<MotorController, MotorController> motors;

  protected abstract double getBaseSpeed();

  protected abstract Pair<MotorController, MotorController> initMotors();

  protected WCDualSubsystem() {
    this.motors = initMotors();
  }

  @Override
  public void periodic() {
    motors.getFirst().setVoltage(speed1 * baseVoltage * getBaseSpeed());
    motors.getSecond().setVoltage(speed2 * baseVoltage * getBaseSpeed());
  }
}
