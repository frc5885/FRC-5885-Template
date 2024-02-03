package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Beambreak;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax m_left;
  private CANSparkMax m_right;
  private double m_speed = 0.5;
  private Beambreak m_beambreak;

  /** Creates a new Shooter. */
  public IntakeSubsystem(Beambreak m_beambreak) {
    m_left = new CANSparkMax(Constants.kIntakeLeft, MotorType.kBrushless);
    m_right = new CANSparkMax(Constants.kIntakeRight, MotorType.kBrushless);
    this.m_beambreak = m_beambreak;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_beambreak.isBroken()) {
      m_left.setVoltage(0);
      m_right.setVoltage(0);
    } else {
      m_left.setVoltage(m_speed * 12);
      m_right.setVoltage(m_speed * -12);
    }
  }
}
