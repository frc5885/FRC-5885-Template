package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.base.RobotSystem;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

// NEXT STEPS
// add encoder limits
// maybe set position function

public class ArmSubsystem extends WCStaticSubsystem {

  // Buffer is value slightly above 0 to ensure doesn't smack
  private final double buffer = 0.008;

  private TalonFX m_arm;
  private DutyCycleEncoder m_encoder;
  private PIDController m_PidController;
  private double m_setPoint;

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_arm = new TalonFX(Constants.kArm);
    m_encoder = new DutyCycleEncoder(1);
    m_PidController = new PIDController(250, 0, 0);
    Logger.SmartDashboard.putData("ArmPID", m_PidController);
    return List.of(m_arm);
  }

  @Override
  public void periodic() {
    super.periodic();
    if (subsystemAction == SubsystemAction.POS) {
      double measurement = getArmPosition();
      m_arm.setVoltage(-m_PidController.calculate(measurement, m_setPoint));
    } else {
      stopMotors();
    }
    positionSim -= m_arm.getMotorVoltage().getValueAsDouble() * 0.001;
    SmartDashboard.putNumber("ArmPosition", getArmPosition());
  }

  @Override
  protected void putDebugDataPeriodic(boolean isRealRobot) {
    Logger.SmartDashboard.putNumber("ArmVoltage", m_arm.getMotorVoltage().getValueAsDouble());
    Logger.SmartDashboard.putNumber("ArmPosition", getArmPosition());
    Logger.SmartDashboard.putNumber("ArmCurrent", m_arm.getSupplyCurrent().getValueAsDouble());
    Logger.SmartDashboard.putString("ArmAction", getActionName());
  }

  private boolean withinUpperLimit() {
    double armPosition = getArmPosition();
    return armPosition < Constants.kArmEncoderMax + buffer;
  }

  private boolean withinLowerLimit() {
    double armPosition = getArmPosition();
    return armPosition > Constants.kArmEncoderMin - buffer;
  }

  public void pos(double setpoint) {
    m_setPoint = setpoint;
    subsystemAction = SubsystemAction.POS;
  }

  private double getArmPosition() {
    return RobotSystem.isReal() ? m_encoder.getAbsolutePosition() : positionSim;
  }

  public boolean isArmDown() {
    return getArmPosition() <= Constants.kArmStow + buffer;
  }

  public boolean isArmUp() {
    return getArmPosition() >= Constants.kArmAmp - buffer;
  }
}
