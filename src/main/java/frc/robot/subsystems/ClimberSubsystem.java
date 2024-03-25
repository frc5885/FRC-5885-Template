package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.WCLogger;
import frc.robot.base.RobotSystem;
import frc.robot.base.subsystems.WCDualSubsystem;

public class ClimberSubsystem extends WCDualSubsystem {

  // TODO ADD NAVX INTEGRATION WHERE IT AUTO BALANCES
  // RN MAKE SECOND CONTROLLER JOYSTICK Y AXIS

  double buffer = 10.0;

  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private RelativeEncoder m_leftRelativeEncoder;
  private RelativeEncoder m_rightRelativeEncoder;

  public ClimberSubsystem() {
    super();
  }

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  @Override
  protected Pair<MotorController, MotorController> initMotors() {
    m_leftMotor = new CANSparkMax(Constants.kClimberLeft, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(Constants.kClimberRight, MotorType.kBrushless);
    m_leftRelativeEncoder = m_leftMotor.getEncoder();
    m_rightRelativeEncoder = m_rightMotor.getEncoder();
    resetEncoders();
    return Pair.of(m_leftMotor, m_rightMotor);
  }

  public void resetEncoders() {
    m_leftRelativeEncoder.setPosition(0.0);
    m_rightRelativeEncoder.setPosition(0.0);
  }

  @Override
  protected void putDebugDataPeriodic(boolean isRealRobot) {
    WCLogger.putNumber(this, "Left/Voltage", m_leftMotor.getAppliedOutput());
    WCLogger.putNumber(this,"Left/Position", getLeftPosition());
    WCLogger.putNumber(this,"Left/Current", m_leftMotor.getOutputCurrent());

    WCLogger.putNumber(this,"Right/Voltage", m_rightMotor.getAppliedOutput());
    WCLogger.putNumber(this,"Right/Position", getRightPosition());
    WCLogger.putNumber(this,"Right/Current", m_rightMotor.getOutputCurrent());
  }

  public void setLeftClimberSpeed(double leftPosition) {
    double leftEncoderValue = Math.abs(getLeftPosition());
    WCLogger.putNumber(this, "Left/StickPosition", leftPosition);
    boolean armLimited = SmartDashboard.getBoolean("ClimberSubsystem/isLimitsEnabled", true);
    if (!armLimited) {
      speed1 = -leftPosition;
    } else if (leftPosition > 0 && leftEncoderValue <= Constants.kLeftClimberMax) {
      speed1 = -leftPosition;
    } else if (leftPosition < 0 && leftEncoderValue >= Constants.kLeftClimberMin + buffer) {
      speed1 = -leftPosition;
    } else {
      speed1 = 0;
    }
  }

  private double getLeftPosition() {
    return RobotSystem.isReal() ? m_leftRelativeEncoder.getPosition() : positionSim1;
  }

  public void rightStickPosition(double rightPosition) {
    double rightEncoderValue = Math.abs(getRightPosition());
    WCLogger.putNumber(this, "RightStickPosition", rightPosition);
    boolean armLimited = SmartDashboard.getBoolean("ClimberSubsystem/isLimitsEnabled", true);
    if (!armLimited) {
      speed2 = -rightPosition;
    } else if (rightPosition > 0 && rightEncoderValue <= Constants.kRightClimberMax) {
      speed2 = -rightPosition;
    } else if (rightPosition < 0 && rightEncoderValue >= Constants.kRightClimberMin + buffer) {
      speed2 = -rightPosition;
    } else {
      speed2 = 0;
    }
  }

  private double getRightPosition() {
    return RobotSystem.isReal() ? m_rightRelativeEncoder.getPosition() : positionSim2;
  }

  @Override
  public void periodic() {
    super.periodic();
    positionSim1 += m_leftMotor.getAppliedOutput() * 0.3;
    positionSim2 += m_rightMotor.getAppliedOutput() * 0.3;
  }
}
