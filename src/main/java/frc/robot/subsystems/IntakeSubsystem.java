package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

public class IntakeSubsystem extends WCStaticSubsystem {

  private CANSparkMax m_intakeMotor;

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_intakeMotor = new CANSparkMax(Constants.kIntakeRight, MotorType.kBrushless);
    return List.of(m_intakeMotor);
  }

  @Override
  protected void putDebugDataPeriodic(boolean isRealRobot) {
    SmartDashboard.putNumber("IntakeVoltage", m_intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("IntakeCurrent", m_intakeMotor.getOutputCurrent());
    SmartDashboard.putString("IntakeAction", getActionName());
  }

  @Override
  public void periodic() {
    super.periodic();
    if (subsystemAction == SubsystemAction.OUTTAKE) {
      reverseMotors();
    } else if (subsystemAction == SubsystemAction.INTAKE) {
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

  public SubsystemAction getSubsystemAction() {
    return subsystemAction;
  }
}
