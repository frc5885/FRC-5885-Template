package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.io.Beambreak;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

public class IntakeSubsystem extends WCStaticSubsystem {

  private CANSparkMax m_left;
  private CANSparkMax m_right;
  private Beambreak m_beambreak;

  /** Creates a new Shooter. */
  // public IntakeSubsystem(Beambreak m_beambreak) {
  // m_left = new CANSparkMax(Constants.kIntakeLeft, MotorType.kBrushless);
  // m_right = new CANSparkMax(Constants.kIntakeRight, MotorType.kBrushless);
  // this.m_beambreak = m_beambreak;
  // }

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  public IntakeSubsystem(Beambreak m_beambreak) {
    super();
    this.m_beambreak = m_beambreak;
  }

  @Override
  protected List<MotorController> initMotors() {
    // m_left = new CANSparkMax(Constants.kIntakeLeft, MotorType.kBrushless);
    m_right = new CANSparkMax(Constants.kIntakeRight, MotorType.kBrushed);
    // return List.of(m_left, m_right);
    return List.of(m_right);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake", m_right.getAppliedOutput());
    if (subsystemAction == SubsystemAction.OUTTAKE) {
      reverseMotors();
    } else if (subsystemAction == SubsystemAction.INTAKE || m_beambreak.isOpen()) {
      forwardMotors();
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
