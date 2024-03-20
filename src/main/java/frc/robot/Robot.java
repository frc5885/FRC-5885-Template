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

  Beambreak m_beambreak;
  ArmSubsystem m_armSubsystem;
  WristSubsystem m_wristSubsystem;
  FeederSubsystem m_feederSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  ClimberSubsystem m_climberSubsystem;
  ShooterSubsystem m_shooterSubsystem;

  LEDSubsystem m_ledSubsystem;

  @Override
  protected void initComponents() {
    m_beambreak = new Beambreak();
    SmartDashboard.putNumber("SHOOTPOINT", Constants.kWristAmp);
  }

  @Override
  protected void initSubsystems() {
    m_intakeSubsystem = new IntakeSubsystem();
    m_armSubsystem = new ArmSubsystem();
    m_wristSubsystem = new WristSubsystem();
    m_feederSubsystem = new FeederSubsystem();
    m_climberSubsystem = new ClimberSubsystem();
    m_shooterSubsystem = new ShooterSubsystem();
    m_ledSubsystem = new LEDSubsystem(
        m_beambreak, m_shooterSubsystem, () -> getSwerveAction() == SwerveAction.AIMBOTTING);
  }

  @Override
  protected void initAutoCommands() {
    // NAMED COMMANDS
    pathPlannerRegisterNamedCommand(
        "shoot",
        new AutoAimShooterCommand(m_shooterSubsystem, m_feederSubsystem, m_wristSubsystem, m_photonVision,
            m_swervePoseEstimator, m_beambreak));
    pathPlannerRegisterNamedCommand("intake", new AutoIntakeCommand(m_intakeSubsystem, m_feederSubsystem, m_beambreak));
  }

  @Override
  protected void initDriverControllerBindings(DriverController m_driverController) {

    // Intake
    m_driverController
        .getRightBumper()
        .whileTrue(new IntakeCommand(m_beambreak, m_intakeSubsystem, m_feederSubsystem));

    // Arm Up
    m_driverController
        .getAButton()
        .onTrue(new ArmUpCommand(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem));

    // Arm Down
    m_driverController.getBButton().onTrue(new ArmDownCommand(m_armSubsystem, m_wristSubsystem));

    // Spin & Aim Shooter
    m_driverController.scheduleOnLeftTrigger(
        new AimShooterCommand(
            m_driverController,
            m_shooterSubsystem,
            this,
            m_wristSubsystem,
            m_photonVision,
            m_swervePoseEstimator,
            m_beambreak));

    // Shoot
    m_driverController.scheduleOnRightTrigger(
        new ShootCommand(m_feederSubsystem, m_shooterSubsystem, m_beambreak));

    // Face forward
    m_driverController
        .getYButton()
        .onTrue(new InstantCommand(() -> setSwerveAction(SwerveAction.FACEFORWARD)));

    // Face backward
    m_driverController
        .getXButton()
        .onTrue(new InstantCommand(() -> setSwerveAction(SwerveAction.FACEBACKWARD)));

    // Face amp
    m_driverController
        .start()
        .onTrue(new InstantCommand(() -> setSwerveAction(SwerveAction.FACEAMP)));

    // Auto
    // m_driverController.back().onTrue(getAutonomousCommand());
  }

  @Override
  protected void initOperatorControllerBindings(OperatorController m_operatorController) {
    m_climberSubsystem.setDefaultCommand(
        new ClimberCommand(m_climberSubsystem, m_operatorController));

    // Outtake
    m_operatorController
        .getRightBumper()
        .whileTrue(new OuttakeCommand(m_beambreak, m_intakeSubsystem, m_feederSubsystem));
  }

  public void setLEDsTeleop() {
    m_ledSubsystem.setTeleop();
  }
}
