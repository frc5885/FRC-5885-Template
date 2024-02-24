package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
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
    return 0.1;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_arm = new TalonFX(Constants.kArm);
    m_PidController = new PIDController(1.0, 0.0, 0.0);
    m_PidController.enableContinuousInput(0, 2 * Math.PI);
    // m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
    // TalonFXConfiguration config = new TalonFXConfiguration();
    return List.of(m_arm);
  }

  @Override
  public void periodic() {
    // System.out.println("Arm Position" + m_arm.getPosition().getValueAsDouble());
    if (subsystemAction == SubsystemAction.UP && withinUpperLimit()) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.DOWN && withinLowerLimit()) {
      reverseMotors();
    } else if (subsystemAction == SubsystemAction.POS) {
      double measurement =
          RobotSystem.isReal() ? m_arm.getPosition().getValueAsDouble() * 2 * Math.PI : positionSim;
      double setpoint = m_setPoint * 2 * Math.PI;
      m_arm.setVoltage(m_PidController.calculate(measurement, setpoint));
      if (measurement == setpoint) {
        subsystemAction = null;
      }
    } else {
      stopMotors();
    }
    positionSim += m_arm.getMotorVoltage().getValueAsDouble() * 0.02;
    Logger.recordOutput("armOutput", m_arm.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "armPosition", RobotSystem.isReal() ? m_arm.getPosition().getValueAsDouble() : positionSim);
    Logger.recordOutput("armUP", subsystemAction == SubsystemAction.UP);
    Logger.recordOutput("armDown", subsystemAction == SubsystemAction.DOWN);
    Logger.recordOutput("armPos", subsystemAction == SubsystemAction.POS);
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
}
