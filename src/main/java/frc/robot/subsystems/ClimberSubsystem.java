package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
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
  private RelativeEncoder m_leftRelativeEncoder;
  private RelativeEncoder m_rightRelativeEncoder;

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
    m_leftRelativeEncoder = m_left.getEncoder();
    m_rightRelativeEncoder = m_right.getEncoder();
    return Pair.of(m_left, m_right);
  }

  public void leftStickPosition(double leftPosition) {
    System.out.println("Left Climber:" + m_leftRelativeEncoder.getPosition());
    speed1 = MathUtil.applyDeadband(leftPosition, Constants.kOperatorLeftDeadzone);
  }

  public void rightStickPosition(double rightPosition) {
    System.out.println("Right Climber:" + m_rightRelativeEncoder.getPosition());
    speed2 = MathUtil.applyDeadband(rightPosition, Constants.kOperatorRightDeadzone);
  }

  @Override
  protected double getBaseSpeed() {
    return 0.5;
  }
}
