package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.io.Beambreak;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

public class ShooterSubsystem extends WCStaticSubsystem {

  private CANSparkMax m_top;
  private CANSparkMax m_bottom;
  private RelativeEncoder m_topEncoder;
  private RelativeEncoder m_bottomEncoder;

  @Override
  protected double getBaseSpeed() {
    return -0.5;
  }

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    super();
    m_topEncoder = m_top.getEncoder();
    m_bottomEncoder = m_bottom.getEncoder();
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
      m_top.setVoltage((getBaseSpeed() - 0.015) * 12);
      m_bottom.setVoltage(getBaseSpeed() * 12);
    } else if (subsystemAction == SubsystemAction.OUTTAKE) {
      m_top.setVoltage(-12 * 0.2);
      m_bottom.setVoltage(-12 * 0.2);
    } else {
      stopMotors();
    }
  }

  @Override
  protected void putDebugDataPeriodic(boolean isRealRobot) {
    SmartDashboard.putNumber("Shooter1Voltage", m_top.getAppliedOutput());
    SmartDashboard.putNumber("Shooter1Current", m_top.getOutputCurrent());
    SmartDashboard.putNumber("Shooter1Velocity", m_topEncoder.getVelocity());

    SmartDashboard.putNumber("Shooter2Voltage", m_bottom.getAppliedOutput());
    SmartDashboard.putNumber("Shooter2Current", m_bottom.getOutputCurrent());
    SmartDashboard.putNumber("Shooter2Velocity", m_bottomEncoder.getVelocity());

    SmartDashboard.putString("ShooterAction", getActionName());
  }

  public void spinFast() {
    subsystemAction = SubsystemAction.SHOOT;
  }

  public void spinSlow() {
    subsystemAction = SubsystemAction.OUTTAKE;
  }

  public boolean isVelocityTerminal() {
    return m_topEncoder.getVelocity() >= 2600 &&
        m_bottomEncoder.getVelocity() >= 2600;
  }
}
