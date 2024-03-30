package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.WCLogger;
import frc.robot.base.RobotSystem;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

public class WristSubsystem extends WCStaticSubsystem {

  private final double buffer = 0.0;

  private SparkAbsoluteEncoder m_absoluteEncoder;
  private CANSparkMax m_wrist;
  private PIDController m_PidController;
  private ArmFeedforward m_feedForward;
  private double m_setPoint = Constants.kWristStow;
  private Mechanism2d m_mechanism2d;
  private MechanismLigament2d m_Ligament2d;

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_wrist = new CANSparkMax(Constants.kWrist, MotorType.kBrushless);
    m_absoluteEncoder = m_wrist.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_PidController = new PIDController(31.0, 0.0, 0.85);
    m_PidController.setTolerance(0.02);

    m_feedForward = new ArmFeedforward(0, 0.45, 0);
    SmartDashboard.putNumber("WristAngleCorrectionFactorClose", Constants.kWristAngleCorrectionFactorClose);
    SmartDashboard.putNumber("WristAngleCorrectionFactorFar", Constants.kWristAngleCorrectionFactorFar);

    m_mechanism2d = new Mechanism2d(2, 10, new Color8Bit(255, 0, 0));
    m_Ligament2d = new MechanismLigament2d("Wrist", 5, 45, 2, new Color8Bit(255, 0, 0));
    WCLogger.putData(this, "PID", m_PidController);
    SmartDashboard.putData("WristPID", m_PidController);
    return List.of(m_wrist);
  }

  @Override
  public void periodic() {
    super.periodic();
    if (m_setPoint < Constants.kWristEncoderMin || m_setPoint > Constants.kWristEncoderMax) {
      return;
    }
    SmartDashboard.putNumber("WristPosition", m_absoluteEncoder.getPosition());
    // SmartDashboard.putNumber("WristSetpoint", m_setPoint);
    // SmartDashboard.putNumber("WristRaw", getWristPosition());
    // SmartDashboard.putBoolean("WrsitPID/atSetpoint", m_PidController.atSetpoint());
    // m_PidController.setP(SmartDashboard.getNumber("WristPID/p", 0.0));
    // m_PidController.setI(SmartDashboard.getNumber("WristPID/i", 0.0));
    // m_PidController.setD(SmartDashboard.getNumber("WristPID/d", 0.0));
    // SmartDashboard.putNumber("WristVoltage", m_wrist.getAppliedOutput()*RobotController.getBatteryVoltage());

    if (subsystemAction == SubsystemAction.POS) {
      double measurement = getWristPosition();
      // double calc = -m_PidController.calculate(measurement, m_setPoint);
      double offsetRadians = -Math.PI / 6.0;
      double calc = -m_feedForward.calculate(measurement*2*Math.PI + offsetRadians, m_setPoint*2*Math.PI + offsetRadians)- m_PidController.calculate(measurement, m_setPoint);
      m_wrist.setVoltage(calc);
    } else {
      stopMotors();
    }
    positionSim += m_wrist.getAppliedOutput() * 0.005;
  }

  @Override
  protected void putDebugDataPeriodic(boolean isRealRobot) {
    WCLogger.putNumber(this, "Voltage", m_wrist.getAppliedOutput());
    WCLogger.putNumber(this, "Position", getWristPosition());
    WCLogger.putNumber(this, "Current", m_wrist.getOutputCurrent());
    WCLogger.putAction(this, "Action", subsystemAction);
    WCLogger.putNumber(this, "SetPoint", m_setPoint);
    WCLogger.putData(this, "SimMech", m_mechanism2d);
  }

  public void pos(double setpoint) {
    m_setPoint = setpoint;
    subsystemAction = SubsystemAction.POS;
  }

  public double getWristPosition() {
    return RobotSystem.isReal() ? m_absoluteEncoder.getPosition() : positionSim;
  }

  public boolean isStowed() {
    return getWristPosition() >= Constants.kWristStow - 0.003;
  }
}
