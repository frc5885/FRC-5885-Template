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

        m_driverController
                .getAButton()
                .whileTrue(m_wristSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        m_driverController
                .getBButton()
                .whileTrue(m_wristSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        m_driverController
                .getXButton()
                .whileTrue(m_wristSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));

        m_driverController
                .getYButton()
                .whileTrue(m_wristSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        
    }

    @Override
    protected void initOperatorControllerBindings(OperatorController m_operatorController) {

        // // Outtake
        // m_operatorController
        //         .getRightBumper()
        //         .whileTrue(new OuttakeCommand(m_beambreak, m_intakeSubsystem, m_feederSubsystem));

        // // Eject Feeder
        // m_operatorController
        //         .getLeftBumper()
        //         .onTrue(new EjectFeederCommand(m_wristSubsystem, m_feederSubsystem, m_armSubsystem));

        // // Arm up
        // m_operatorController
        //         .getAButton()
        //         .onTrue(
        //                 new ArmUpCommand(
        //                         m_armSubsystem,
        //                         m_wristSubsystem,
        //                         m_shooterSubsystem,
        //                         m_feederSubsystem,
        //                         m_beambreak));

        // // Arm down
        // m_operatorController.getBButton().onTrue(new ArmDownCommand(m_armSubsystem, m_wristSubsystem));

        // // Shooter Aim Override
        // m_operatorController.scheduleOnLeftTriggerTrue(new OverrideShootCommand(
        //     this,
        //     m_operatorController,
        //     m_shooterSubsystem,
        //     m_wristSubsystem,
        //     m_armSubsystem,
        //     m_beambreak));
    }

    public void setLEDsTeleop() {
        m_ledSubsystem.setTeleop();
    }
}
