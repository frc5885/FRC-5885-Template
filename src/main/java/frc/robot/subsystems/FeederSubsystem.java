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

public class FeederSubsystem extends WCStaticSubsystem {

  private CANSparkMax m_feeder;
  private Beambreak m_beambreak;

  @Override
  protected double getBaseSpeed() {
    return 0.5;
  }

  public FeederSubsystem(Beambreak m_beambreak) {
    super();
    this.m_beambreak = m_beambreak;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_feeder = new CANSparkMax(Constants.kFeeder, MotorType.kBrushless);
    return List.of(m_feeder);
  }

  @Override
  public void periodic() {
    if (subsystemAction == SubsystemAction.INTAKE || m_beambreak.isOpen()) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.OUTTAKE) {
      reverseMotors();
    } else {
      stopMotors();
    }
    Logger.recordOutput("feeder", m_feeder.getAppliedOutput());
  }

  public void intake() {
    subsystemAction = SubsystemAction.INTAKE;
  }

  public void outtake() {
    subsystemAction = SubsystemAction.OUTTAKE;
  }
}
