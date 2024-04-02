package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.WCLogger;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;

public class IntakeSubsystem extends WCStaticSubsystem {

  private CANSparkMax m_intakeMotor;
  private LinearFilter m_currentFilter = LinearFilter.singlePoleIIR(0.2, 0.02);


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
    WCLogger.putNumber(this, "Voltage", m_intakeMotor.getAppliedOutput());
    WCLogger.putNumber(this, "Current", m_intakeMotor.getOutputCurrent());
    WCLogger.putAction(this, "Action", subsystemAction);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("THING0", m_intakeMotor.getOutputCurrent());
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

  public double getVelocity() {
    return m_intakeMotor.getEncoder().getVelocity();
  }

  public SubsystemAction getSubsystemAction() {
    return subsystemAction;
  }

  public double getMotorCurrent() {
    return m_currentFilter.calculate(m_intakeMotor.getOutputCurrent());
  }

  public boolean hasNote() {
    return getMotorCurrent() > Constants.kIntakeCurrentThreshold;
  }
}
