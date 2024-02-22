package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import frc.robot.base.io.Beambreak;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends WCStaticSubsystem {

  private CANSparkMax m_left;
  private CANSparkMax m_right;
  private Beambreak m_beambreak;

  /** Creates a new Shooter. */
  // public IntakeSubsystem(Beambreak m_beambreak) {
  //   m_left = new CANSparkMax(Constants.kIntakeLeft, MotorType.kBrushless);
  //   m_right = new CANSparkMax(Constants.kIntakeRight, MotorType.kBrushless);
  //   this.m_beambreak = m_beambreak;
  // }

  @Override
  protected double getBaseSpeed() {
    return 0.5;
  }

  public IntakeSubsystem(Beambreak m_beambreak) {
    super();
    this.m_beambreak = m_beambreak;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_left = new CANSparkMax(Constants.kIntakeLeft, MotorType.kBrushless);
    m_right = new CANSparkMax(Constants.kIntakeRight, MotorType.kBrushless);
    return List.of(m_left, m_right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (m_beambreak.isBroken()) {
    //   m_left.setVoltage(0);
    //   m_right.setVoltage(0);
    // } else {
    //   m_left.setVoltage(m_speed * 12);
    //   m_right.setVoltage(m_speed * -12);
    // }

    if (subsystemAction == SubsystemAction.INTAKE || m_beambreak.isOpen()) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.OUTTAKE) {
      reverseMotors();
    } else {
      stopMotors();
    }
  }

  public void intake() {
    subsystemAction = SubsystemAction.INTAKE;
  }

  public void outtake() {
    subsystemAction = SubsystemAction.OUTTAKE;
  }
}
