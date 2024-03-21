package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.RobotSystem;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

public class WristSubsystem extends WCStaticSubsystem {

  private final double buffer = 0.0;

  private SparkAbsoluteEncoder m_absoluteEncoder;
  private CANSparkMax m_wrist;
  private PIDController m_PidController;
  private double m_setPoint = Constants.kWristStow;

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_wrist = new CANSparkMax(Constants.kWrist, MotorType.kBrushless);
    m_absoluteEncoder = m_wrist.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_PidController = new PIDController(35, 10, 0.5);
    SmartDashboard.putData("WristPID", m_PidController);
    return List.of(m_wrist);
  }

  @Override
  public void periodic() {
    super.periodic();
    if (m_setPoint < Constants.kWristEncoderMin || m_setPoint > Constants.kWristEncoderMax) {
      return;
    }

    if (subsystemAction == SubsystemAction.POS) {
      double measurement = getWristPosition();
      double calc = m_PidController.calculate(measurement, m_setPoint);
      m_wrist.setVoltage(calc);
    } else {
      stopMotors();
    }
    positionSim += m_wrist.getAppliedOutput() * 0.005;
  }

  @Override
  protected void putDebugDataPeriodic(boolean isRealRobot) {
    SmartDashboard.putNumber("WristVoltage", m_wrist.getAppliedOutput());
    SmartDashboard.putNumber("WristPosition", getWristPosition());
    SmartDashboard.putNumber("WristCurrent", m_wrist.getOutputCurrent());
    SmartDashboard.putString("WristAction", getActionName());
    SmartDashboard.putNumber("WristSetPoint", m_setPoint);
  }

  public void pos(double setpoint) {
    m_setPoint = setpoint;
    subsystemAction = SubsystemAction.POS;
  }

  public double getWristPosition() {
    return RobotSystem.isReal() ? m_absoluteEncoder.getPosition() : positionSim;
  }

  public boolean isStowed() {
    return getWristPosition() >= Constants.kWristStow;
  }

  @Override
  public void stop() {
    super.stop();
    double d = 0;
  }
}
