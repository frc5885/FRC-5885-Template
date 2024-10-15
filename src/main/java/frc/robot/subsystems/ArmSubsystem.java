package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.WCLogger;
import frc.robot.base.RobotSystem;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// NEXT STEPS
// add encoder limits
// maybe set position function

public class ArmSubsystem extends WCStaticSubsystem {

  // Buffer is value slightly above 0 to ensure doesn't smack
  private final double buffer = 0.008;
  private final double simArmStow = 0;

  private TalonFX m_arm;
  static DutyCycleEncoder m_encoder;
  private PIDController m_PidController;
  private double m_setPoint;
  private Pose3d m_armSim;
  private Pose3d m_armBaseSim;

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_arm = new TalonFX(Constants.kArm);
    m_encoder = new DutyCycleEncoder(1);
    m_PidController = new PIDController(250, 0, 0);
    // m_armSim = new Pose3d();
    m_armSim = new Pose3d(0.346, -0.0009, 0.298, new Rotation3d(0, 0, 0));

    return List.of(m_arm);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("ArmRaw", getArmPosition());
    SmartDashboard.putNumber(
        "armpidnuber", -m_PidController.calculate(getArmPosition(), m_setPoint));
    super.periodic();
    if (subsystemAction == SubsystemAction.POS) {
      double measurement = getArmPosition();
      m_arm.setVoltage(-m_PidController.calculate(measurement, m_setPoint));
    } else {
      stopMotors();
    }
    positionSim -= m_arm.getMotorVoltage().getValueAsDouble() * 0.001;

    m_armSim =
        m_armSim.plus(
            new Transform3d(
                0,
                0,
                0,
                new Rotation3d(
                    0, -m_arm.getMotorVoltage().getValueAsDouble() * 0.0015 * Math.PI, 0)));
  }

  @Override
  protected void putDebugDataPeriodic(boolean isRealRobot) {
    WCLogger.putNumber(this, "Voltage", m_arm.getMotorVoltage().getValueAsDouble());
    WCLogger.putNumber(this, "Position", getArmPosition());
    WCLogger.putNumber(this, "Current", m_arm.getSupplyCurrent().getValueAsDouble());
    WCLogger.putAction(this, "Action", subsystemAction);
    if (WCLogger.isEnabled) {
      Logger.recordOutput("ArmSim", m_armSim);
      Logger.recordOutput("ArmBaseSim", m_armBaseSim);
    }
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
    return RobotSystem.isReal()
        ? getArmPosition() <= Constants.kArmStow
        : getArmPosition() <= simArmStow + buffer;
    // return getArmPosition() <= Constants.kArmStow ;
  }

  public boolean isArmUp() {
    return getArmPosition() >= Constants.kArmAmp - buffer;
  }

  public double getArmAngleSim() {
    return m_armSim.getRotation().getY();
  }
}
