package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  // private double m_leftEncoderOffset;
  // private double m_rightEncoderOffset;

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
    // m_leftEncoderOffset = m_leftRelativeEncoder.getPosition();
    // m_rightEncoderOffset = m_rightRelativeEncoder.getPosition();
    return Pair.of(m_left, m_right);
  }

  public void resetEncoders(){
    m_leftRelativeEncoder.setPosition(0.0);
    m_rightRelativeEncoder.setPosition(0.0);
  }

  public void leftStickPosition(double leftPosition) {
    // System.out.println("Left Climber:" + m_leftRelativeEncoder.getPosition());
    double leftEncoderValue = m_leftRelativeEncoder.getPosition()*-1;
    SmartDashboard.putNumber("ClimberLeft", leftEncoderValue);
    leftPosition = MathUtil.applyDeadband(leftPosition, Constants.kOperatorLeftDeadzone);
    // if (leftPosition < 0 && m_leftRelativeEncoder.getPosition() > Constants.kLeftClimberMin){
    //   speed1 = leftPosition;
    // }
    // else if (leftPosition > 0 && m_leftRelativeEncoder.getPosition() <
    // Constants.kLeftClimberMax){
    //   speed1 = leftPosition;
    // }
    // else{
    //   speed1 = 0;
    // }
    if (leftPosition < 0 && leftEncoderValue <= Constants.kLeftClimberMax) {
      speed1 = leftPosition;
    }
    // // trying to move down and encoder is above min
    else if (leftPosition > 0
        && leftEncoderValue >= Constants.kLeftClimberMin) {
      speed1 = leftPosition;
    } else {
      speed1 = 0;
    }
  }

  public void rightStickPosition(double rightPosition) {
    double rightEncoderValue = m_rightRelativeEncoder.getPosition()*-1;
    SmartDashboard.putNumber("ClimberRight", rightEncoderValue);
    // System.out.println("Right Climber:" + rightEncoderValue);
    rightPosition = MathUtil.applyDeadband(rightPosition, Constants.kOperatorRightDeadzone);
    // speed2 = rightPosition;

    //  right position is joystick value
    // getposition starts at 0
    // up is negative encoder value

    // trying to move up and encoder is below max
    if (rightPosition < 0 && rightEncoderValue <= Constants.kRightClimberMax) {
      speed2 = rightPosition;
    }
    // // trying to move down and encoder is above min
    else if (rightPosition > 0
        && rightEncoderValue >= Constants.kRightClimberMin) {
      speed2 = rightPosition;
    } else {
      speed2 = 0;
    }
  }

  @Override
  protected double getBaseSpeed() {
    return 0.5;
  }
}
