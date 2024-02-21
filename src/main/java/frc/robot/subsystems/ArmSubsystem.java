package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

// NEXT STEPS
// add encoder limits
// maybe set position function

public class ArmSubsystem extends WCStaticSubsystem {

  // Buffer is value slightly above 0 to ensure doesn't smack
  private final double buffer = 0.0;

  private TalonFX m_arm;

  @Override
  protected double getBaseSpeed() {
    return 0.1;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_arm = new TalonFX(Constants.kArm);
    // m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
    // TalonFXConfiguration config = new TalonFXConfiguration();
    return List.of(m_arm);
  }

  @Override
  public void periodic() {
    //    System.out.println("Arm Position" + m_arm.getPosition().getValueAsDouble());
    if (subsystemAction == SubsystemAction.UP && !isAtUpperLimit()) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.DOWN && !isAtLowerLimit()) {
      reverseMotors();
    } else {
      stopMotors();
    }
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
}
