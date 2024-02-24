package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import frc.robot.base.io.Beambreak;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

public class ShooterSubsystem extends WCStaticSubsystem {

  private CANSparkMax m_top;
  private CANSparkMax m_bottom;
  // private double m_speed = 0.25;
  private Beambreak m_beambreak;

  @Override
  protected double getBaseSpeed() {
    return -0.5;
  }

  /** Creates a new Shooter. */
  public ShooterSubsystem(Beambreak m_beambreak) {
    super();
    // m_top = new CANSparkMax(Constants.kShooterTop, MotorType.kBrushless);
    // m_bottom = new CANSparkMax(Constants.kShooterBottom, MotorType.kBrushless);
    this.m_beambreak = m_beambreak;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_top = new CANSparkMax(Constants.kShooterTop, MotorType.kBrushless);
    m_bottom = new CANSparkMax(Constants.kShooterBottom, MotorType.kBrushless);
    return List.of(m_top, m_bottom);
  }

  @Override
  public void periodic() {

    if (m_beambreak.isBroken()) {
      forwardMotors();
    } else {
      stopMotors();
    }
  }
}
