package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.*;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.RobotSystem;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;

import java.beans.Encoder;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// NEXT STEPS
// add encoder limits
// maybe set position function

public class ArmSubsystem extends WCStaticSubsystem {

  // Buffer is value slightly above 0 to ensure doesn't smack
  private final double buffer = 0.0;

  private TalonFX m_arm;

  private PIDController m_PidController;
  private double m_setPoint;
  private DutyCycleEncoder m_encoder;

  @Override
  protected double getBaseSpeed() {
    return 0.3;
  }

  @Override
  protected Double getBaseSpeedDown() {
    return 0.1;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_arm = new TalonFX(Constants.kArm);
    m_encoder = new DutyCycleEncoder(1);
    
    m_PidController = new PIDController(1.0, 5.0, 0.05);
    // m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
    // TalonFXConfiguration config = new TalonFXConfiguration();
    return List.of(m_arm);
  }

  @Override
  public void periodic() {
    // System.out.println("Arm Position" + m_arm.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Arm", m_encoder.getAbsolutePosition());
    if (subsystemAction == SubsystemAction.UP /*&& withinLowerLimit()/* */) {
      reverseMotors();
    } else if (subsystemAction == SubsystemAction.DOWN /*&& withinUpperLimit()/* */) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.POS) {
      double measurement =
          RobotSystem.isReal() ? m_encoder.getAbsolutePosition() : positionSim;
      m_arm.setVoltage(m_PidController.calculate(measurement, m_setPoint));
      double buffer = 0.5;
      if (measurement <= m_setPoint + buffer && measurement >= m_setPoint - buffer) {
        subsystemAction = null;
      }
    } else {
      stopMotors();
    }
        SmartDashboard.putNumber("ArmVoltage", m_arm.getMotorVoltage().getValueAsDouble());

    positionSim += m_arm.getMotorVoltage().getValueAsDouble() * 0.02;
    Logger.recordOutput("armOutput", m_arm.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "armPosition", RobotSystem.isReal() ? m_encoder.getAbsolutePosition() : positionSim);
  }

  private boolean withinUpperLimit() {
    double armPosition =
        RobotSystem.isReal() ? m_encoder.getAbsolutePosition() : positionSim;
    return armPosition < Constants.kArmEncoderMax + buffer;
  }

  private boolean withinLowerLimit() {
    double armPosition =
        RobotSystem.isReal() ? m_encoder.getAbsolutePosition() : positionSim;
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
}
