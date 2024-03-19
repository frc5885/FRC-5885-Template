package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.RobotSystem;
import frc.robot.base.subsystems.WCDualSubsystem;

public class ClimberSubsystem extends WCDualSubsystem {

  // TODO ADD NAVX INTEGRATION WHERE IT AUTO BALANCES
  // RN MAKE SECOND CONTROLLER JOYSTICK Y AXIS

  double buffer = 2.0;

  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private RelativeEncoder m_leftRelativeEncoder;
  private RelativeEncoder m_rightRelativeEncoder;

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
    SmartDashboard.putNumber("ClimberLeftVoltage", m_leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("ClimberLeftPosition", getLeftPosition());
    SmartDashboard.putNumber("ClimberLeftCurrent", m_leftMotor.getOutputCurrent());

    SmartDashboard.putNumber("ClimberRightVoltage", m_rightMotor.getAppliedOutput());
    SmartDashboard.putNumber("ClimberRightPosition", getRightPosition());
    SmartDashboard.putNumber("ClimberRightCurrent", m_rightMotor.getOutputCurrent());
  }

  public void setLeftClimberSpeed(double leftPosition) {
    double leftEncoderValue = getLeftPosition();
    SmartDashboard.putNumber("ClimberLeftStickPosition", leftPosition);
    if (leftPosition > 0 && leftEncoderValue <= Constants.kLeftClimberMax) {
      speed1 = leftPosition;
    } else if (leftPosition < 0 && leftEncoderValue >= Constants.kLeftClimberMin + buffer) {
      speed1 = leftPosition;
    } else {
      speed1 = 0;
    }
  }

  private double getLeftPosition() {
    return RobotSystem.isReal() ? m_leftRelativeEncoder.getPosition() : positionSim1;
  }

  public void rightStickPosition(double rightPosition) {
    double rightEncoderValue = getRightPosition();
    SmartDashboard.putNumber("ClimberRightStickPosition", rightPosition);
    if (rightPosition > 0 && rightEncoderValue <= Constants.kRightClimberMax) {
      speed2 = rightPosition;
    } else if (rightPosition < 0 && rightEncoderValue >= Constants.kRightClimberMin + buffer) {
      speed2 = rightPosition;
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
