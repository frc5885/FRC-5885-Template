package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.WCLogger;
import frc.robot.base.RobotSystem;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends WCStaticSubsystem {

  private final double buffer = 0.003;

  private SparkAbsoluteEncoder m_absoluteEncoder;
  private CANSparkMax m_wrist;
  private PIDController m_PidController;
  private ArmFeedforward m_feedForward;
  private double m_setPoint = Constants.kWristStow;
  private Pose3d m_wristSim;

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
    SmartDashboard.putNumber(
        "WristAngleCorrectionFactorClose", Constants.kWristAngleCorrectionFactorClose);
    SmartDashboard.putNumber(
        "WristAngleCorrectionFactorFar", Constants.kWristAngleCorrectionFactorFar);

    positionSim = Constants.kWristStow;
    m_wristSim =
        new Pose3d(-0.096, -0.003, 0.344, new Rotation3d(0, Units.degreesToRadians(60), Math.PI));
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
    // SmartDashboard.putNumber("WristVoltage",
    // m_wrist.getAppliedOutput()*RobotController.getBatteryVoltage());

    if (subsystemAction == SubsystemAction.POS) {
      double measurement = getWristPosition();
      // double calc = -m_PidController.calculate(measurement, m_setPoint);
      double offsetRadians = -Math.PI / 6.0;
      double calc =
          -m_feedForward.calculate(
                  measurement * 2 * Math.PI + offsetRadians,
                  m_setPoint * 2 * Math.PI + offsetRadians)
              - m_PidController.calculate(measurement, m_setPoint);
      m_wrist.setVoltage(calc);
    } else {
      stopMotors();
    }
    positionSim += m_wrist.getAppliedOutput() * -0.005;
    m_wristSim =
        m_wristSim.plus(
            new Transform3d(
                0, 0, 0, new Rotation3d(0, m_wrist.getAppliedOutput() * -0.005 * 2 * Math.PI, 0)));
  }

  @Override
  protected void putDebugDataPeriodic(boolean isRealRobot) {
    WCLogger.putNumber(this, "Voltage", m_wrist.getAppliedOutput());
    WCLogger.putNumber(this, "Position", getWristPosition());
    WCLogger.putNumber(this, "Current", m_wrist.getOutputCurrent());
    WCLogger.putAction(this, "Action", subsystemAction);
    WCLogger.putNumber(this, "SetPoint", m_setPoint);
    if (WCLogger.isEnabled) {
      Logger.recordOutput("WristSim", m_wristSim);
    }
  }

  public void pos(double setpoint) {
    m_setPoint = setpoint;
    subsystemAction = SubsystemAction.POS;
  }

  public double getWristPosition() {
    return RobotSystem.isReal() ? m_absoluteEncoder.getPosition() : positionSim;
  }

  public Rotation3d getWristSimRotation() {
    return m_wristSim.getRotation();
  }

  public boolean isStowed() {
    return getWristPosition() >= Constants.kWristStow - buffer;
  }

  public boolean isAtPos() {
    return getWristPosition() >= m_setPoint - buffer && getWristPosition() <= m_setPoint + buffer;
  }
}
