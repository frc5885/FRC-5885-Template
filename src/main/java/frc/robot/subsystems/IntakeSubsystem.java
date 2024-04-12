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

  private CANSparkMax m_intakeMotorRight;
  private CANSparkMax m_intakeMotorLeft;
  // Never touch 0.02
  private LinearFilter m_currentFilter = LinearFilter.singlePoleIIR(0.5, 0.02);

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_intakeMotorRight = new CANSparkMax(Constants.kIntakeRight, MotorType.kBrushless);
    m_intakeMotorLeft = new CANSparkMax(Constants.kIntakeLeft, MotorType.kBrushless);
    m_intakeMotorLeft.setInverted(true);
    return List.of(m_intakeMotorRight, m_intakeMotorLeft);
  }

  @Override
  protected void putDebugDataPeriodic(boolean isRealRobot) {
    WCLogger.putNumber(this, "Voltage", m_intakeMotorRight.getAppliedOutput());
    WCLogger.putNumber(this, "Current", m_intakeMotorRight.getOutputCurrent());
    WCLogger.putAction(this, "Action", subsystemAction);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("THING0", m_intakeMotorRight.getOutputCurrent());
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
    return m_intakeMotorRight.getEncoder().getVelocity();
  }

  public SubsystemAction getSubsystemAction() {
    return subsystemAction;
  }

  public double getMotorCurrent() {
    double current = m_currentFilter.calculate(m_intakeMotorRight.getOutputCurrent());
    // SmartDashboard.putNumber("l )INTAKECURRENT", current);
    return current;
  }

  public boolean hasNote() {
    return getMotorCurrent() > Constants.kIntakeCurrentThreshold;
  }
}
