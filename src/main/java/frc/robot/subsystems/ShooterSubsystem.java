package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import frc.robot.base.io.Beambreak;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

public class ShooterSubsystem extends WCStaticSubsystem {

  private CANSparkMax m_top;
  private CANSparkMax m_bottom;
  // private double m_speed = 0.25;
  private Beambreak m_beambreak;
  private ArmSubsystem m_armSubsystem;

  private RelativeEncoder m_topEncoder;
  private RelativeEncoder m_bottomEncoder;

  @Override
  protected double getBaseSpeed() {
    return -0.5;
  }

  /** Creates a new Shooter. */
  public ShooterSubsystem(Beambreak m_beambreak, ArmSubsystem armSubsystem) {
    super();
    m_armSubsystem = armSubsystem;
    // m_top = new CANSparkMax(Constants.kShooterTop, MotorType.kBrushless);
    // m_bottom = new CANSparkMax(Constants.kShooterBottom, MotorType.kBrushless);
    this.m_beambreak = m_beambreak;
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
    if (subsystemAction == SubsystemAction.SHOOT) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.OUTTAKE) {
      m_top.setVoltage(-12 * 0.2);
      m_bottom.setVoltage(-12 * 0.2);
    } else {
      stopMotors();
    }
  }

  public void spinFast() {
    subsystemAction = SubsystemAction.SHOOT;
  }

  public void spinSlow() {
    subsystemAction = SubsystemAction.OUTTAKE;
  }
}
