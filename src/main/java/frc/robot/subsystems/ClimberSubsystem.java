package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  enum ClimberAction {
    ON,
    OFF;
  }

  // TODO ADD NAVX INTEGRATION WHERE IT AUTO BALANCES
  // RN MAKE SECOND CONTROLLER JOYSTICK Y AXIS

  private ClimberAction leftStickAction = ClimberAction.OFF;
  private ClimberAction rightStickAction = ClimberAction.OFF;

  private CANSparkMax m_left;
  private CANSparkMax m_right;

  private double m_leftPosition;
  private double m_rightPosition;

  /** Creates a new Shooter. */
  public ClimberSubsystem() {
    // m_leftPosition = leftPosition;
    // m_rightPosition = rightPosition;
    m_left = new CANSparkMax(Constants.kClimberLeft, MotorType.kBrushless);
    m_right = new CANSparkMax(Constants.kClimberRight, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // if (rightStickAction == ClimberAction.ON) {
    //     m_right.setVoltage(//joystick position * 12);
    // } else if (leftStickAction == ClimberAction.ON) {
    //     m_left.setVoltage(// joystick postion * 12);
    // } else {
    //     m_left.setVoltage(0.0);
    //     m_right.setVoltage(0.0);
    // }
  }

  // public void leftStickButton(boolean isPressed) {
  //     if (isPressed) {
  //         leftStickAction = ClimberAction.ON;
  //     } else {
  //         leftStickAction = ClimberAction.OFF;
  //     }
  // }

  // public void rightStickButton(boolean isPressed) {
  //     if (isPressed) {
  //         rightStickAction = ClimberAction.ON;
  //     } else {
  //         rightStickAction = ClimberAction.OFF;
  //     }
  // }

  public void climb(double leftPosition, double rightPosition, double speedFactor) {
    m_left.setVoltage(leftPosition * 12.0 * speedFactor);
    m_right.setVoltage(rightPosition * 12.0 * speedFactor);
  }
}
