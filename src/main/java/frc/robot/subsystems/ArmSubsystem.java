package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.RobotSystem;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
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
    m_PidController = new PIDController(1.0, 5.0, 0.05);
    // m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
    // TalonFXConfiguration config = new TalonFXConfiguration();
    resetEncoder();
    return List.of(m_arm);
  }

  @Override
  public void periodic() {
    // System.out.println("Arm Position" + m_arm.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Arm", m_arm.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("ArmVoltage", m_arm.getMotorVoltage().getValueAsDouble());
    if (subsystemAction == SubsystemAction.UP && withinLowerLimit()) {
      reverseMotors();
    } else if (subsystemAction == SubsystemAction.DOWN && withinUpperLimit()) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.POS) {
      double measurement =
          RobotSystem.isReal() ? m_arm.getPosition().getValueAsDouble() : positionSim;
      m_arm.setVoltage(m_PidController.calculate(measurement, m_setPoint));
      double buffer = 0.5;
      if (measurement <= m_setPoint + buffer && measurement >= m_setPoint - buffer) {
        subsystemAction = null;
      }
    } else {
      stopMotors();
    }
    positionSim += m_arm.getMotorVoltage().getValueAsDouble() * 0.02;
    Logger.recordOutput("armOutput", m_arm.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "armPosition", RobotSystem.isReal() ? m_arm.getPosition().getValueAsDouble() : positionSim);
  }

  private boolean withinUpperLimit() {
    double armPosition =
        RobotSystem.isReal() ? m_arm.getPosition().getValueAsDouble() : positionSim;
    return armPosition < Constants.kArmEncoderMax + buffer;
  }

  private boolean withinLowerLimit() {
    double armPosition =
        RobotSystem.isReal() ? m_arm.getPosition().getValueAsDouble() : positionSim;
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

  public void resetEncoder() {
    m_arm.setPosition(0.0);
  }
}
