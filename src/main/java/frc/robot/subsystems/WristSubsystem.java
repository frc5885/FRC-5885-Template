package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// NEXT STEPS
// add encoder limits
// maybe set position function

public class WristSubsystem extends SubsystemBase {

  private TalonFX m_arm;
  private double m_speed = 0.1;
  private Boolean m_isReversed;

  /** Creates a new Shooter. */
  public WristSubsystem() {
    m_arm = new TalonFX(Constants.kWrist);
    // m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
    TalonFXConfiguration config = new TalonFXConfiguration();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_arm.setVoltage(m_speed * 12.0);
  }

  public void moveWrist(boolean isReversed) {
    if (isReversed) {
      m_speed = -m_speed;
    }
  }
}
