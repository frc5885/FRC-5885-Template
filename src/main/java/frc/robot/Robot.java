package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.base.WCRobot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.DriverController;
import frc.robot.base.io.OperatorController;
import frc.robot.base.subsystems.swerve.SwerveAction;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

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
  }

  @Override
  protected void initSubsystems() {
    SmartDashboard.putNumber("Wrist shoot point", Constants.kWristStow);
    m_intakeSubsystem = new IntakeSubsystem();
    m_armSubsystem = new ArmSubsystem();
    m_wristSubsystem = new WristSubsystem();
    m_feederSubsystem = new FeederSubsystem();
    m_climberSubsystem = new ClimberSubsystem();
    m_shooterSubsystem = new ShooterSubsystem();
    m_ledSubsystem = new LEDSubsystem(m_beambreak, () -> getSwerveAction() == SwerveAction.AIMBOTTING);
  }

  @Override
  protected void initAutoCommands() {
    // NAMED COMMANDS
    pathPlannerRegisterNamedCommand(
        "shoot",
        new AutoShootCommand(
            this,
            m_feederSubsystem,
            m_intakeSubsystem,
            m_wristSubsystem,
            m_swerveDrive,
            m_swervePoseEstimator,
            m_photonVision));
  }

  @Override
  protected void initDriverControllerBindings(
      DriverController m_driverController, OperatorController operatorController) {

    // Intake
    m_driverController
        .getRightBumper()
            .whileTrue(new IntakeCMD(m_beambreak, m_intakeSubsystem, m_feederSubsystem));

    // Arm Up
    m_driverController
        .getAButton()
        .onTrue(new ArmUpCmd(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem));

    // Arm Down
    m_driverController
        .getBButton()
        .onTrue(new ArmDownCmd(m_armSubsystem, m_wristSubsystem));

    // Spin & Aim Shooter
    m_driverController.scheduleOnLeftTrigger(
        new AimShooterCommand(m_driverController, m_shooterSubsystem)
    );

    // Shoot
    m_driverController.scheduleOnRightTrigger(
        new ShootCommand(m_feederSubsystem)
    );

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
  }

  @Override
  protected void initOperatorControllerBindings(
      DriverController driverController, OperatorController m_operatorController) {
      m_climberSubsystem.setDefaultCommand(
        new ClimberCommand(m_climberSubsystem, m_operatorController)
      );
  }

  @Override
  public void setLEDsTeleop() {
    m_ledSubsystem.setTeleop();
  }
}
