package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import frc.robot.base.subsystems.WCDualSubsystem;

public class ClimberSubsystem extends WCDualSubsystem {

  //   enum ClimberAction {
  //     ON,
  //     OFF;
  //   }

  // TODO ADD NAVX INTEGRATION WHERE IT AUTO BALANCES
  // RN MAKE SECOND CONTROLLER JOYSTICK Y AXIS

  private CANSparkMax m_left;
  private CANSparkMax m_right;

  /** Creates a new Shooter. */
  public ClimberSubsystem() {
    // m_leftPosition = leftPosition;
    // m_rightPosition = rightPosition;
    // m_left = new CANSparkMax(Constants.kClimberLeft, MotorType.kBrushless);
    // m_right = new CANSparkMax(Constants.kClimberRight, MotorType.kBrushless);
    super();
  }

  @Override
  protected Pair<MotorController, MotorController> initMotors() {
    m_left = new CANSparkMax(Constants.kClimberLeft, MotorType.kBrushless);
    m_right = new CANSparkMax(Constants.kClimberRight, MotorType.kBrushless);
    return Pair.of(m_left, m_right);
  }

  public void leftStickPosition(double leftPosition) {
    if (Math.abs(leftPosition) > Constants.kOperatorLeftDeadzone) {
      speed1 = leftPosition;
    } else {
      speed1 = 0.0;
    }
  }

  public void rightStickPosition(double rightPosition) {
    if (Math.abs(rightPosition) > Constants.kOperatorRightDeadzone) {
      speed2 = rightPosition;
    } else {
      speed2 = 0.0;
    }
  }

  @Override
  protected double getBaseSpeed() {
    return 0.5;
  }
}
