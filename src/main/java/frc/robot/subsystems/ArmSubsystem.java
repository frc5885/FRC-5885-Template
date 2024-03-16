package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.RobotSystem;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

// NEXT STEPS
// add encoder limits
// maybe set position function

public class ArmSubsystem extends WCStaticSubsystem {

  // Buffer is value slightly above 0 to ensure doesn't smack
  private final double buffer = 0.008;

  private TalonFX m_arm;
  private DutyCycleEncoder m_encoder;
  private PIDController m_PidController;
  private double m_setPoint;

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  @Override
  protected Double getBaseSpeedDown() {
    return 0.1;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_arm = new TalonFX(Constants.kArm);
    m_encoder = new DutyCycleEncoder(1);
    m_PidController = new PIDController(250, 0, 0);
    SmartDashboard.putData("ArmPID", m_PidController);
    // m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
    // TalonFXConfiguration config = new TalonFXConfiguration();
    return List.of(m_arm);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ArmVoltage", m_arm.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("ArmPosition", m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("ArmCurrent", m_arm.getSupplyCurrent().getValueAsDouble());
    if (subsystemAction == SubsystemAction.UP /* && withinLowerLimit() */) {
      reverseMotors();
    } else if (subsystemAction == SubsystemAction.DOWN /* && withinUpperLimit() */) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.POS) {
      double measurement = RobotSystem.isReal() ? m_encoder.getAbsolutePosition() : positionSim;
      m_arm.setVoltage(-m_PidController.calculate(measurement, m_setPoint));
      double buffer = 0.005;
      if (measurement <= m_setPoint + buffer && measurement >= m_setPoint - buffer) {
        subsystemAction = null;
      }
    } else {
      stopMotors();
    }
    positionSim += m_arm.getMotorVoltage().getValueAsDouble() * 0.02;
  }

  private boolean withinUpperLimit() {
    double armPosition = RobotSystem.isReal() ? m_encoder.getAbsolutePosition() : positionSim;
    return armPosition < Constants.kArmEncoderMax + buffer;
  }

  private boolean withinLowerLimit() {
    double armPosition = RobotSystem.isReal() ? m_encoder.getAbsolutePosition() : positionSim;
    return armPosition > Constants.kArmEncoderMin - buffer;
  }

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

  public boolean isArmDown() {
    return m_encoder.getAbsolutePosition() <= Constants.kArmStow + buffer;
  }

  public boolean isArmUp() {
    return m_encoder.getAbsolutePosition() >= Constants.kArmAmp - buffer;
  }

  public boolean isArmSetUp() {
    return m_setPoint == Constants.kArmAmp;
  }
}
