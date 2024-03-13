package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.base.WCRobot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.DriverController;
import frc.robot.base.io.OperatorController;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IntakeCMD;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpinShooterCMD;
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
    m_intakeSubsystem = new IntakeSubsystem(m_beambreak);
    m_armSubsystem = new ArmSubsystem();
    m_wristSubsystem = new WristSubsystem();
    m_feederSubsystem = new FeederSubsystem(m_beambreak);
    m_climberSubsystem = new ClimberSubsystem();
    m_shooterSubsystem = new ShooterSubsystem(m_beambreak, m_armSubsystem);
    m_ledSubsystem = new LEDSubsystem(m_beambreak, () -> isAimBotting());
  }

  @Override
  protected void initAutoCommands() {
    // NAMED COMMANDS
    pathPlannerRegisterNamedCommand(
        "shoot",
        new AutoShootCommand(
            this,
            m_feederSubsystem,
            m_wristSubsystem,
            m_swerveDrive,
            m_swervePoseEstimator,
            m_photonVision));
  }

  @Override
  protected void initDriverControllerBindings(DriverController m_driverController, OperatorController operatorController) {

    m_feederSubsystem.setDefaultCommand(new ShootCommand(m_feederSubsystem, operatorController, m_driverController));
    m_intakeSubsystem.setDefaultCommand(new IntakeCMD(m_beambreak, m_intakeSubsystem, m_feederSubsystem, m_driverController, operatorController));
    m_driverController
        .getAButton()
        .onTrue(new InstantCommand(
            () -> {
              m_armSubsystem.pos(Constants.kArmAmp);
              m_wristSubsystem.pos(Constants.kWristAmp);
            }));

    m_driverController
        .getBButton()
        .onTrue(new InstantCommand(() -> {
          m_armSubsystem.pos(Constants.kArmStow);
          m_wristSubsystem.pos(Constants.kWristStow);
        }));

    m_driverController
        .getRightTriggerAxis();
  }

  @Override
  protected void initOperatorControllerBindings(DriverController driverController, OperatorController m_operatorController) {


    m_climberSubsystem.setDefaultCommand(
        new ClimberCommand(m_climberSubsystem, m_operatorController));

    m_feederSubsystem.setDefaultCommand(new ShootCommand(m_feederSubsystem, m_operatorController, driverController));

    m_shooterSubsystem.setDefaultCommand(new SpinShooterCMD(m_operatorController, m_shooterSubsystem, m_armSubsystem));
  
  }

  @Override
  public void setLEDsTeleop() {
    m_ledSubsystem.setTeleop();
  }
}
