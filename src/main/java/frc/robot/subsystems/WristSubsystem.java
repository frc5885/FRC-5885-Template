package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;

import java.util.List;

// NEXT STEPS
// add encoder limits
// maybe set position function

public class WristSubsystem extends WCStaticSubsystem {

  // Buffer is value slightly above 0 to ensure doesn't smack
  private final double buffer = 0.0;

  private TalonFX m_wrist;

  @Override
  protected double getBaseSpeed() {
    return 0.1;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_wrist = new TalonFX(Constants.kWrist);
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
    } else {
      stopMotors();
    }
  }

  private boolean isAtUpperLimit() {
    double armPosition = m_wrist.getPosition().getValueAsDouble();
    return armPosition >= Constants.kWristEncoderMax + buffer;
  }

  private boolean isAtLowerLimit() {
    double armPosition = m_wrist.getPosition().getValueAsDouble();
    return armPosition <= Constants.kWristEncoderMin - buffer;
  }

  public void forward() {
    subsystemAction = SubsystemAction.FORWARD;
  }

  public void reverse() {
    subsystemAction = SubsystemAction.REVERSE;
  }
}
