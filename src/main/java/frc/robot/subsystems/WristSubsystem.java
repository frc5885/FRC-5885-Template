package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// NEXT STEPS
// add encoder limits
// maybe set position function

public class WristSubsystem extends SubsystemBase {

  enum WristAction {
    ON,
    OFF;
  }

  private WristAction startAction = WristAction.OFF;
  private WristAction backAction = WristAction.OFF;

  private TalonFX m_wrist;
  private double m_speed = 0.1;

  /** Creates a new Shooter. */
  public WristSubsystem() {
    m_wrist = new TalonFX(Constants.kWrist);
    // m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
    // TalonFXConfiguration config = new TalonFXConfiguration();
  }

  @Override
  public void periodic() {
    if (startAction == WristAction.ON) {
      m_wrist.setVoltage(m_speed * 12.0);
    } else if (backAction == WristAction.ON) {
      m_wrist.setVoltage(m_speed * 12.0);
    } else {
      m_wrist.setVoltage(0.0);
    }
  }

  public void startButton(boolean isPressed) {
    if (isPressed) {
      startAction = WristAction.ON;
    } else {
      startAction = WristAction.OFF;
    }
  }

  public void backButton(boolean isPressed) {
    if (isPressed) {
      backAction = WristAction.ON;
    } else {
      backAction = WristAction.OFF;
    }
  }
}
