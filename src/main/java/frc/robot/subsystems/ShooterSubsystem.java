package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.WCLogger;
import frc.robot.base.RobotSystem;
import frc.robot.base.io.Beambreak;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

public class ShooterSubsystem extends WCStaticSubsystem {

  private CANSparkMax m_top;
  private CANSparkMax m_bottom;
  private RelativeEncoder m_topEncoder;
  private RelativeEncoder m_bottomEncoder;
  Beambreak m_beambreak;
  double topVelocitySim = 0.0;
  double bottomVelocitySim = 0.0;

  double idleVelocity = -2800;
  double shootCloseVelocity = -5000;
  double shootFarVelocity = -3500;
  double passCloseVelocity = -2800;
  double passFarVelocity = -3700;
  double passVelocity = -2800;
  public RobotMode robotMode = RobotMode.AUTO;

  public enum RobotMode {
    TELEOP,
    AUTO
  }

  // Bad one
  PIDController m_topPIDController = new PIDController(0.005, 0.001, 0.0);
  SimpleMotorFeedforward m_topFeedforward = new SimpleMotorFeedforward(0.0, 0.002377, 0.0);

  // Good one
  PIDController m_bottomPIDController = new PIDController(0.003, 0.0, 0.0);
  SimpleMotorFeedforward m_bottomFeedforward = new SimpleMotorFeedforward(0.0, 0.002311, 0.0);

  @Override
  protected double getBaseSpeed() {
    return -0.7;
  }

  /** Creates a new Shooter. */
  public ShooterSubsystem(Beambreak beambreak) {
    super();
    m_topEncoder = m_top.getEncoder();
    m_bottomEncoder = m_bottom.getEncoder();
    WCLogger.putData(this, "Top/PID", m_topPIDController);
    WCLogger.putData(this, "Bottom/PID", m_bottomPIDController);
    m_beambreak = beambreak;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_top = new CANSparkMax(Constants.kShooterTop, MotorType.kBrushless);
    m_bottom = new CANSparkMax(Constants.kShooterBottom, MotorType.kBrushless);
    return List.of(m_top, m_bottom);
  }

  @Override
  public void periodic() {
    super.periodic();
    if (subsystemAction == SubsystemAction.SHOOT) {
      m_top.setVoltage((-0.7 - 0.015) * 12);
      m_bottom.setVoltage(-0.7 * 12);
    } else if (subsystemAction == SubsystemAction.SHOOT_FAR) {
      double setVoltage =
          MathUtil.clamp(
              m_topPIDController.calculate(getTopVelocity(), getBottomVelocity())
                  + m_topFeedforward.calculate(shootFarVelocity),
              -12,
              0);
      double setVoltage2 =
          MathUtil.clamp(
              m_bottomPIDController.calculate(getBottomVelocity(), shootFarVelocity)
                  + m_bottomFeedforward.calculate(shootFarVelocity),
              -12,
              0);
      m_top.setVoltage(setVoltage);
      m_bottom.setVoltage(setVoltage2);
    } else if (subsystemAction == SubsystemAction.SHOOT_CLOSE) {
      double setVoltage =
          MathUtil.clamp(
              m_topPIDController.calculate(getTopVelocity(), getBottomVelocity())
                  + m_topFeedforward.calculate(shootCloseVelocity),
              -12,
              0);
      double setVoltage2 =
          MathUtil.clamp(
              m_bottomPIDController.calculate(getBottomVelocity(), shootCloseVelocity)
                  + m_bottomFeedforward.calculate(shootCloseVelocity),
              -12,
              0);
      m_top.setVoltage(setVoltage);
      m_bottom.setVoltage(setVoltage2);
    } else if (subsystemAction == SubsystemAction.OUTTAKE) {
      m_top.setVoltage(-12 * 0.2);
      m_bottom.setVoltage(-12 * 0.2);
    } else if (robotMode == RobotMode.TELEOP && m_beambreak.isBroken()) {
      double setVoltage =
          MathUtil.clamp(
              m_topPIDController.calculate(getTopVelocity(), getBottomVelocity())
                  + m_topFeedforward.calculate(idleVelocity),
              -12,
              0);
      double setVoltage2 =
          MathUtil.clamp(
              m_bottomPIDController.calculate(getBottomVelocity(), idleVelocity)
                  + m_bottomFeedforward.calculate(idleVelocity),
              -12,
              0);
      m_top.setVoltage(setVoltage);
      m_bottom.setVoltage(setVoltage2);
    } else if (subsystemAction == SubsystemAction.PASS) {
      double setVoltage =
          MathUtil.clamp(
              m_topPIDController.calculate(getTopVelocity(), getBottomVelocity())
                  + m_topFeedforward.calculate(passVelocity),
              -12,
              0);
      double setVoltage2 =
          MathUtil.clamp(
              m_bottomPIDController.calculate(getBottomVelocity(), passVelocity)
                  + m_bottomFeedforward.calculate(passVelocity),
              -12,
              0);
      m_top.setVoltage(setVoltage);
      m_bottom.setVoltage(setVoltage2);
    } else {
      stopMotors();
      topVelocitySim = 0;
      bottomVelocitySim = 0;
    }
    if (!isVelocityTerminal()) {
      topVelocitySim += m_top.getAppliedOutput() * 1.75;
      bottomVelocitySim += m_bottom.getAppliedOutput() * 1.75;
    }
    SmartDashboard.putNumber("Top/Voltage", m_top.getAppliedOutput());
    SmartDashboard.putNumber("Top/Current", m_top.getOutputCurrent());
    SmartDashboard.putNumber("Top/Velocity", getTopVelocity());

    SmartDashboard.putNumber("Bottom/Voltage", m_bottom.getAppliedOutput());
    SmartDashboard.putNumber("Bottom/Current", m_bottom.getOutputCurrent());
    SmartDashboard.putNumber("Bottom/Velocity", getBottomVelocity());
  }

  @Override
  protected void putDebugDataPeriodic(boolean isRealRobot) {
    WCLogger.putNumber(this, "Top/Voltage", m_top.getAppliedOutput());
    WCLogger.putNumber(this, "Top/Current", m_top.getOutputCurrent());
    WCLogger.putNumber(this, "Top/Velocity", getTopVelocity());

    WCLogger.putNumber(this, "Bottom/Voltage", m_bottom.getAppliedOutput());
    WCLogger.putNumber(this, "Bottom/Current", m_bottom.getOutputCurrent());
    WCLogger.putNumber(this, "Bottom/Velocity", getBottomVelocity());

    WCLogger.putAction(this, "Action", subsystemAction);
  }

  public void spinFastFar(double velocity) {
    subsystemAction = SubsystemAction.SHOOT_FAR;
    shootFarVelocity = velocity;
  }

  public void spinFastClose() {
    subsystemAction = SubsystemAction.SHOOT_CLOSE;
  }

  public void spinSlow() {
    subsystemAction = SubsystemAction.OUTTAKE;
  }

  public void spinFast() {
    subsystemAction = SubsystemAction.SHOOT;
  }

  public void spinPass() {
    subsystemAction = SubsystemAction.PASS;
  }

  public boolean isVelocityTerminal() {
    double velocity;
    if (subsystemAction == SubsystemAction.SHOOT_CLOSE) {
      velocity = shootCloseVelocity + 200;
    } else {
      velocity = shootFarVelocity + 200;
    }
    return getTopVelocity() <= velocity && getBottomVelocity() <= velocity;
  }

  private double getTopVelocity() {
    return RobotSystem.isReal() ? m_topEncoder.getVelocity() : topVelocitySim;
  }

  private double getBottomVelocity() {
    return RobotSystem.isReal() ? m_bottomEncoder.getVelocity() : bottomVelocitySim;
  }
}
