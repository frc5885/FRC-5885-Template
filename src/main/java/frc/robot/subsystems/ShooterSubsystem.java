package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Beambreak;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax m_top;
  private CANSparkMax m_bottom;
  private double m_speed = 0.25;
  private Beambreak m_beambreak;

  /** Creates a new Shooter. */
  public ShooterSubsystem(Beambreak m_beambreak) {
    m_top = new CANSparkMax(Constants.kShooterTop, MotorType.kBrushless);
    m_bottom = new CANSparkMax(Constants.kShooterBottom, MotorType.kBrushless);
    this.m_beambreak = m_beambreak;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!m_beambreak.isBroken()) {
      m_top.setVoltage(0);
      m_bottom.setVoltage(0);
    } else {
      m_top.setVoltage(m_speed * 12);
      m_bottom.setVoltage(m_speed * -12);
    }
  }
}
