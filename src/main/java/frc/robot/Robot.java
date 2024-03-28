package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.base.WCRobot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.DriverController;
import frc.robot.base.io.OperatorController;
import frc.robot.base.subsystems.swerve.SwerveAction;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Robot extends WCRobot {

    public Beambreak m_beambreak;
    public ArmSubsystem m_armSubsystem;
    public WristFFSubsystem m_wristSubsystem;
    public FeederSubsystem m_feederSubsystem;
    public IntakeSubsystem m_intakeSubsystem;
    ClimberSubsystem m_climberSubsystem;
    public ShooterSubsystem m_shooterSubsystem;

    LEDSubsystem m_ledSubsystem;

    public Robot() {
        SmartDashboard.putBoolean("ClimberSubsystem/isLimitsEnabled", true);
    }

    @Override
    protected void initComponents() {
        m_beambreak = new Beambreak();
    }

    @Override
    protected void initSubsystems() {
        m_intakeSubsystem = new IntakeSubsystem();
        m_armSubsystem = new ArmSubsystem();
        m_wristSubsystem = new WristFFSubsystem();
        m_feederSubsystem = new FeederSubsystem();
        m_climberSubsystem = new ClimberSubsystem();
        m_shooterSubsystem = new ShooterSubsystem(m_beambreak);
        m_ledSubsystem = new LEDSubsystem(
                m_beambreak, m_shooterSubsystem, () -> getSwerveAction() == SwerveAction.AIMBOTTING);
    }

    @Override
    protected void initAutoCommands() {}

    @Override
    protected void initDriverControllerBindings(DriverController m_driverController) {

        
    }

    @Override
    protected void initOperatorControllerBindings(OperatorController m_operatorController) {

        m_operatorController
                .getAButton()
                .whileTrue(m_wristSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        m_operatorController
                .getBButton()
                .whileTrue(m_wristSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        m_operatorController
                .getXButton()
                .whileTrue(m_wristSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));

        m_operatorController
                .getYButton()
                .whileTrue(m_wristSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public void setLEDsTeleop() {
        m_ledSubsystem.setTeleop();
    }
}
