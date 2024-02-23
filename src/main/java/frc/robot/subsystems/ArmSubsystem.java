package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
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
  private Rotation2d m_SetPoint;

  @Override
  protected double getBaseSpeed() {
    return 0.1;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_arm = new TalonFX(Constants.kArm);
    m_PidController = new PIDController(5.0, 0.0, 0.0);
    m_PidController.enableContinuousInput(-Math.PI, Math.PI);
    // m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
    // TalonFXConfiguration config = new TalonFXConfiguration();
    return List.of(m_arm);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("armmoving", false);
    // System.out.println("Arm Position" + m_arm.getPosition().getValueAsDouble());
    if (subsystemAction == SubsystemAction.UP && !isAtUpperLimit()) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.DOWN && !isAtLowerLimit()) {
      reverseMotors();
    } else if (subsystemAction == SubsystemAction.POS && !isAtUpperLimit() && !isAtLowerLimit()) {
      m_arm.setVoltage(m_PidController.calculate(m_arm.getPosition().getValue() * 2 * Math.PI, m_SetPoint.getRadians()));
      Logger.recordOutput("armmoving", true);
    } else {
      stopMotors();
    }
    Logger.recordOutput("armPosition", m_arm.getPosition().getValueAsDouble());
  }

  private boolean isAtUpperLimit() {
    double armPosition = m_arm.getPosition().getValueAsDouble();
    return armPosition >= Constants.kArmEncoderMax + buffer;
  }

  private boolean isAtLowerLimit() {
    double armPosition = m_arm.getPosition().getValueAsDouble();
    return armPosition <= Constants.kArmEncoderMin - buffer;
  }

  public void up() {
    subsystemAction = SubsystemAction.UP;
  }

  public void down() {
    subsystemAction = SubsystemAction.DOWN;
  }

  public void toPos(Rotation2d desired) {
    subsystemAction = SubsystemAction.POS;
    m_SetPoint = desired;
  }
}
