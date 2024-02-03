package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Beambreak;

public class FeederSubsystem extends SubsystemBase {

  private CANSparkMax m_feeder;
  private double m_speed = 0.5;
  private Beambreak m_beambreak;

  /** Creates a new Shooter. */
    public FeederSubsystem(Beambreak m_beambreak) {
    m_feeder = new CANSparkMax(Constants.kFeeder, MotorType.kBrushless);
    this.m_beambreak = m_beambreak;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_beambreak.isBroken()) {
      m_feeder.setVoltage(0);
    } else {
      m_feeder.setVoltage(m_speed * 12);
    }
  }
}