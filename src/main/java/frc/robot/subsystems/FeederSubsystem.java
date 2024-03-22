package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

public class FeederSubsystem extends WCStaticSubsystem {

  WPI_TalonSRX m_feeder;

  @Override
  protected double getBaseSpeed() {
    return -0.3;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_feeder = new WPI_TalonSRX(Constants.kFeeder);
    return List.of(m_feeder);
  }

  @Override
  public void periodic() {
    super.periodic();
    if (subsystemAction == SubsystemAction.INTAKE) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.OUTTAKE) {
      reverseMotors();
    } else if (subsystemAction == SubsystemAction.EJECT) {
      m_feeder.setVoltage(12.0);
    } else {
      stopMotors();
    }
  }

  @Override
  protected void putDebugDataPeriodic(boolean isRealRobot) {
    SmartDashboard.putNumber("FeederVoltage", m_feeder.getMotorOutputVoltage());
    SmartDashboard.putNumber("FeederCurrent", m_feeder.getStatorCurrent());
    SmartDashboard.putString("FeederAction", getActionName());
  }

  public void intake() {
    subsystemAction = SubsystemAction.INTAKE;
  }

  public void outtake() {
    subsystemAction = SubsystemAction.OUTTAKE;
  }

  public void eject() {
    subsystemAction = SubsystemAction.EJECT;
  }
}
