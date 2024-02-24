package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

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

public class WristSubsystem extends WCStaticSubsystem {

  // Buffer is value slightly above 0 to ensure doesn't smack
  private final double buffer = 0.0;

  private TalonFX m_wrist;
  private PIDController m_PidController;


  @Override
  protected double getBaseSpeed() {
    return 0.1;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_wrist = new TalonFX(Constants.kWrist);
    m_PidController = new PIDController(1.0, 0.0, 0.0);
    m_PidController.enableContinuousInput(-Math.PI, Math.PI);
    // m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
    // TalonFXConfiguration config = new TalonFXConfiguration();
    return List.of(m_wrist);
  }

  @Override
  public void periodic() {
    if (subsystemAction == SubsystemAction.FORWARD && !isAtUpperLimit()) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.REVERSE && !isAtLowerLimit()) {
      reverseMotors();
    } else if (subsystemAction == SubsystemAction.POS) {
      double measurement =
          RobotSystem.isReal() ? m_wrist.getPosition().getValueAsDouble() * 2 * Math.PI : positionSim;
      double setpoint = Constants.kSetPoint.getRadians();
      m_wrist.setVoltage(m_PidController.calculate(measurement, setpoint));
      if (measurement == setpoint) {
        subsystemAction = null;
      }
    } else {
      stopMotors();
    }
    positionSim += m_wrist.getMotorVoltage().getValueAsDouble() * 0.02;
    Logger.recordOutput("armOutput", m_wrist.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "armPosition", RobotSystem.isReal() ? m_wrist.getPosition().getValueAsDouble() : positionSim);
    Logger.recordOutput("UP", subsystemAction == SubsystemAction.UP);
    Logger.recordOutput("Down", subsystemAction == SubsystemAction.DOWN);
    Logger.recordOutput("Pos", subsystemAction == SubsystemAction.POS);
  }

  private boolean isAtUpperLimit() {
    double wristPosition =
      RobotSystem.isReal() ? m_wrist.getPosition().getValueAsDouble() : positionSim;
    return wristPosition < Constants.kWristEncoderMax + buffer;
  }

  private boolean isAtLowerLimit() {
    double wristPosition =
      RobotSystem.isReal() ? m_wrist.getPosition().getValueAsDouble() : positionSim;
    return wristPosition > Constants.kWristEncoderMin - buffer;
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

  public void pos() {
    subsystemAction = SubsystemAction.POS;
  }
}
