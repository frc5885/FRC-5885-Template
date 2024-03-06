package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.RobotSystem;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// Code for neo is currently commented out
// Look at inital neo code for wrist commit to see how to use the throughbore encoder

public class WristSubsystem extends WCStaticSubsystem {

  // Buffer is value slightly above 0 to ensure doesn't smack
  private final double buffer = 0.075;

  private SparkAbsoluteEncoder m_absoluteEncoder;
  private CANSparkMax m_wrist;
  // private SparkLimitSwitch m_limitSwitchForward;
  // private SparkLimitSwitch m_limitSwitchReverse;
  private PIDController m_PidController;
  private double m_setPoint;

  @Override
  protected double getBaseSpeed() {
    return 0.5;
  }

  @Override
  protected List<MotorController> initMotors() {
    // m_wrist = new CANSparkMax(Constants.kWrist, MotorType.kBrushless);
    m_wrist = new CANSparkMax(Constants.kWrist, MotorType.kBrushless);
    // m_limitSwitchForward = m_wrist.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    // m_limitSwitchReverse = m_wrist.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_absoluteEncoder = m_wrist.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_PidController = new PIDController(15.0, 2.5, 1.0);
    m_PidController.setTolerance(0.1);
    SmartDashboard.putData("WristPID", m_PidController);

    // addChild("PID", m_PidController);
    // m_PidController.enableContinuousInput(0, 2 * Math.PI);
    // What resetEncoders() does has also been commented out
    // resetEncoders();
    // m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
    // TalonFXConfiguration config = new TalonFXConfiguration();
    startPID(); // this will start a PID with setpoint at current position when robot starts up
    return List.of(m_wrist);
  }

  // LOWER LIMIT ISNT ZERO IT WILL START IN AN IN BETWEEN
  @Override
  public void periodic() {

    // SmartDashboard.putNumber("Wrist", m_wrist.getEncoder().getPosition());
    // SmartDashboard.putNumber("Wrist", m_wrist.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
    SmartDashboard.putNumber("Wrist", m_absoluteEncoder.getPosition());
    SmartDashboard.putNumber("WristVoltage", m_wrist.getAppliedOutput());
    // SmartDashboard.putBoolean("Limit Forward", m_limitSwitchForward.isPressed());
    // SmartDashboard.putBoolean("Limit Reverse", m_limitSwitchReverse.isPressed());
    SmartDashboard.putNumber("Wrist setPoint", m_setPoint);
    // System.out.println("Wrist Position" + m_wrist.getPosition().getValueAsDouble());
    if (subsystemAction == SubsystemAction.UP && isAtUpperLimit()) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.DOWN
        && isAtLowerLimit() /*&& m_limitSwitchForward.isPressed() */) {
      reverseMotors();
    } else if (subsystemAction == SubsystemAction.POS) {
      double measurement = RobotSystem.isReal() ? m_absoluteEncoder.getPosition() : positionSim;
      double setpoint = m_setPoint;
      m_wrist.setVoltage(m_PidController.calculate(measurement, setpoint));
      // if (measurement <= m_setPoint + buffer && measurement >= m_setPoint - buffer) {
      //   subsystemAction = null;
      // }
    } else {
      stopMotors();
    }
    positionSim += m_wrist.getAppliedOutput() * 0.02;
    Logger.recordOutput("WristOutput", m_wrist.getAppliedOutput());
    Logger.recordOutput(
        "wristPosition", RobotSystem.isReal() ? m_absoluteEncoder.getPosition() : positionSim);
  }

  private boolean isAtUpperLimit() {
    double wristPosition = RobotSystem.isReal() ? m_absoluteEncoder.getPosition() : positionSim;
    return wristPosition < Constants.kWristEncoderMax + buffer;
  }

  private boolean isAtLowerLimit() {
    double wristPosition = RobotSystem.isReal() ? m_absoluteEncoder.getPosition() : positionSim;
    return wristPosition > Constants.kWristEncoderMin - buffer;
  }

  @Override
  public void stop() {
    // lock PID to current position
    subsystemAction = SubsystemAction.POS;
    m_setPoint = m_absoluteEncoder.getPosition();
  }

  public void startPID() {
    // start PID to current position when robot turns on
    m_setPoint = m_absoluteEncoder.getPosition();
    subsystemAction = SubsystemAction.POS;
  }

  // public void forward() {
  //   subsystemAction = SubsystemAction.FORWARD;
  // }

  // public void reverse() {
  //   subsystemAction = SubsystemAction.REVERSE;
  // }

  public void up() {
    subsystemAction = SubsystemAction.UP;
  }

  public void down() {
    subsystemAction = SubsystemAction.DOWN;
  }

  public void pos(double setpoint) {
    m_setPoint = setpoint;
    subsystemAction = SubsystemAction.POS;
  }

  public void resetEncoders() {
    // m_wrist.getEncoder().setPosition(0.0);
  }
}
