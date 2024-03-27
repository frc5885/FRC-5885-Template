package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    public WristSubsystem m_wristSubsystem;
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
        m_wristSubsystem = new WristSubsystem();
        m_feederSubsystem = new FeederSubsystem();
        m_climberSubsystem = new ClimberSubsystem();
        m_shooterSubsystem = new ShooterSubsystem(m_beambreak);
        m_ledSubsystem = new LEDSubsystem(
                m_beambreak, m_shooterSubsystem, () -> getSwerveAction() == SwerveAction.AIMBOTTING);
    }

    @Override
    protected void initAutoCommands() {
        // NAMED COMMANDS
        pathPlannerRegisterNamedCommand(
                "shoot",
                new AutoAimShooterCommand(
                        this,
                        m_swerveDrive,
                        m_shooterSubsystem,
                        m_feederSubsystem,
                        m_wristSubsystem,
                        m_photonVision,
                        m_swervePoseEstimator,
                        m_beambreak));
    }

    @Override
    protected void initDriverControllerBindings(DriverController m_driverController) {

        // Intake
        m_driverController
                .getRightBumper()
                .whileTrue(
                        new IntakeCommand(
                                m_beambreak,
                                m_intakeSubsystem,
                                m_feederSubsystem,
                                m_wristSubsystem,
                                m_armSubsystem));

        // Arm Up, snap to amp
        m_driverController
                .getAButton()
                .whileTrue(
                        new ArmUpSnapCommand(
                                m_armSubsystem,
                                m_wristSubsystem,
                                m_shooterSubsystem,
                                m_feederSubsystem,
                                m_beambreak,
                                this))
                .onFalse(new ArmDownCommand(m_armSubsystem, m_wristSubsystem));

        // Spin & Aim Shooter
        m_driverController.scheduleOnLeftTriggerTrue(
                new ShootCommand(
                        this,
                        m_driverController,
                        m_shooterSubsystem,
                        m_wristSubsystem,
                        m_armSubsystem,
                        m_photonVision,
                        m_swervePoseEstimator,
                        m_beambreak));

        // Aim Wrist
        m_driverController.scheduleOnLeftTriggerFalse(
                new DefaultWristAimCommand(
                        m_driverController, 
                        this, 
                        m_armSubsystem, 
                        m_wristSubsystem, 
                        m_photonVision, 
                        m_swervePoseEstimator, 
                        m_beambreak));

        // Aim Note
        m_driverController
                .getLeftBumper()
                .whileTrue(new SetSwerveActionCommand(this, SwerveAction.AIMNOTE));

        // Shoot
        m_driverController.scheduleOnRightTrigger(
                new FeedCommand(
                        m_feederSubsystem, m_shooterSubsystem, m_armSubsystem, m_wristSubsystem, m_beambreak));

        // snap to amp
        m_driverController
                .getStartButton()
                .whileTrue(new SetSwerveActionCommand(this, SwerveAction.FACEAMP));

        // snap to source
        m_driverController
                .getBButton()
                .whileTrue(new SetSwerveActionCommand(this, SwerveAction.FACESOURCE));

        // Face forward
        m_driverController
                .getYButton()
                .onTrue(new InstantCommand(() -> setSwerveAction(SwerveAction.FACEFORWARD)));

        // Face backward
        m_driverController
                .getXButton()
                .onTrue(new InstantCommand(() -> setSwerveAction(SwerveAction.FACEBACKWARD)));
    }

    @Override
    protected void initOperatorControllerBindings(OperatorController m_operatorController) {
        m_climberSubsystem.setDefaultCommand(
                new ClimberCommand(m_climberSubsystem, m_operatorController));

        // Outtake
        m_operatorController
                .getRightBumper()
                .whileTrue(new OuttakeCommand(m_beambreak, m_intakeSubsystem, m_feederSubsystem));

        // Eject Feeder
        m_operatorController
                .getLeftBumper()
                .onTrue(new EjectFeederCommand(m_wristSubsystem, m_feederSubsystem, m_armSubsystem));

        // Arm up
        m_operatorController
                .getAButton()
                .onTrue(
                        new ArmUpCommand(
                                m_armSubsystem,
                                m_wristSubsystem,
                                m_shooterSubsystem,
                                m_feederSubsystem,
                                m_beambreak));

        // Arm down
        m_operatorController.getBButton().onTrue(new ArmDownCommand(m_armSubsystem, m_wristSubsystem));

        // Shooter Aim Override
        m_operatorController.scheduleOnLeftTriggerTrue(new OverrideShootCommand(
            this,
            m_operatorController,
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_beambreak));
    }

    public void setLEDsTeleop() {
        m_ledSubsystem.setTeleop();
    }
}
