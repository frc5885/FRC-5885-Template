package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
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

public class WristSubsystem extends WCStaticSubsystem {

  // Buffer is value slightly above 0 to ensure doesn't smack
  private final double buffer = 0.0;

  private CANSparkMax m_wrist;
  private PIDController m_PidController;
  private double m_setPoint;

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_wrist = new CANSparkMax(Constants.kWrist, MotorType.kBrushless);
    m_PidController = new PIDController(1.0, 0.0, 0.0);
    m_PidController.enableContinuousInput(0, 2 * Math.PI);
    resetEncoders();
    // m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
    // TalonFXConfiguration config = new TalonFXConfiguration();
    return List.of(m_wrist);
  }

  // LOWER LIMIT ISNT ZERO IT WILL START IN AN IN BETWEEN
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist", m_wrist.getEncoder().getPosition());
    // System.out.println("Wrist Position" + m_wrist.getPosition().getValueAsDouble());
    if (subsystemAction == SubsystemAction.UP && isAtUpperLimit()) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.DOWN && isAtLowerLimit()) {
      reverseMotors();
    } else if (subsystemAction == SubsystemAction.POS) {
      double measurement =
          RobotSystem.isReal() ? m_wrist.getEncoder().getPosition() * 2 * Math.PI : positionSim;
      double setpoint = m_setPoint * 2 * Math.PI;
      m_wrist.setVoltage(m_PidController.calculate(measurement, setpoint));
      if (measurement == setpoint) {
        subsystemAction = null;
      }
    } else {
      stopMotors();
    }
    positionSim += m_wrist.getAppliedOutput() * 0.02;
    Logger.recordOutput("wristOutput", m_wrist.getAppliedOutput());
    Logger.recordOutput(
        "wristPosition", RobotSystem.isReal() ? m_wrist.getEncoder().getPosition() : positionSim);
    Logger.recordOutput("wristUP", subsystemAction == SubsystemAction.UP);
    Logger.recordOutput("wristDown", subsystemAction == SubsystemAction.DOWN);
    Logger.recordOutput("wristPos", subsystemAction == SubsystemAction.POS);
  }

  private boolean isAtUpperLimit() {
    double wristPosition = RobotSystem.isReal() ? m_wrist.getEncoder().getPosition() : positionSim;
    return wristPosition < Constants.kWristEncoderMax + buffer;
  }

  private boolean isAtLowerLimit() {
    double wristPosition = RobotSystem.isReal() ? m_wrist.getEncoder().getPosition() : positionSim;
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

  public void pos(double setpoint) {
    m_setPoint = setpoint;
    subsystemAction = SubsystemAction.POS;
  }

  public void resetEncoders() {
    m_wrist.getEncoder().setPosition(0.0);
  }
}
