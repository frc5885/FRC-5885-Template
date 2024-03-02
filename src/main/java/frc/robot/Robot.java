package frc.robot;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.base.WCRobot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.DriverController;
import frc.robot.base.io.OperatorController;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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

  @Override
  protected void initComponents() {
    m_beambreak = new Beambreak();
  }

  @Override
  protected void initSubsystems() {
    m_intakeSubsystem = new IntakeSubsystem(m_beambreak);
    m_armSubsystem = new ArmSubsystem();
    m_wristSubsystem = new WristSubsystem();
    m_feederSubsystem = new FeederSubsystem(m_beambreak);
    m_climberSubsystem = new ClimberSubsystem();
    m_shooterSubsystem = new ShooterSubsystem(m_beambreak);
  }

  @Override
  protected void initDriverControllerBindings(DriverController m_driverController) {

    m_feederSubsystem.setDefaultCommand(new ShootCommand(m_feederSubsystem, m_driverController));

    // m_driverController
    //     .getYButton()
    //     .whileTrue(
    //         new StartEndCommand(() -> m_feederSubsystem.outtake(), () -> m_feederSubsystem.stop()));

    m_driverController
        .getYButton()
        .whileTrue(new InstantCommand(() -> m_wristSubsystem.pos(Constants.kWristEject)));

    // aimbot mode (toggle on/off)
    m_driverController
        .getLeftBumper()
        .whileTrue(new InstantCommand(() -> setAimBotting(!isAimBotting())));

    m_driverController
        .getAButton()
        .whileTrue(new InstantCommand(() -> m_armSubsystem.pos(Constants.kArmAmp)));

    // For testing
    // m_driverController
    //     .getRightBumper()
    //     .whileTrue(
    //         new StartEndCommand(() -> m_intakeSubsystem.outtake(), () ->
    // m_intakeSubsystem.stop()));
    // new StartEndCommand(() -> m_isAimbotting = true, () -> m_isAimbotting = false));

    // m_driverController
    //     .getYButton()
    //     .whileTrue();

    // new SequentialCommandGroup(
    //     new InstantCommand(() -> m_wristSubsystem.pos(Constants.kWristEject)),
    //     new
    //     new InstantCommand()
    // );

    // Intake
    // m_driverController
    //     .getXButton()
    //     .whileTrue(
    //         new StartEndCommand(() -> m_intakeSubsystem.outtake(), () -> m_intakeSubsystem.stop()));

    // // Arm - Up
    // m_driverController
    //     .getRightBumper()
    //     .whileTrue(new StartEndCommand(() -> m_armSubsystem.up(), () -> m_armSubsystem.stop()));

    // // Arm - Down
    // m_driverController
    //     .getLeftBumper()
    //     .whileTrue(new StartEndCommand(() -> m_armSubsystem.down(), () ->
    // m_armSubsystem.stop()));

    // // Wrist - Forward
    // m_driverController
    //     .getStartButton()
    //     .whileTrue(new StartEndCommand(() -> m_wristSubsystem.up(), () ->
    // m_wristSubsystem.stop()));

    // // Wrist - Reverse
    // m_driverController
    //     .getBackButton()
    //     .whileTrue(
    //         new StartEndCommand(() -> m_wristSubsystem.down(), () -> m_wristSubsystem.stop()));
  }

  @Override
  protected void initOperatorControllerBindings(OperatorController m_operatorController) {

    m_climberSubsystem.setDefaultCommand(
        new ClimberCommand(m_climberSubsystem, m_operatorController));

    m_feederSubsystem.setDefaultCommand(new ShootCommand(m_feederSubsystem, m_operatorController));

    // Arm ToPos
    // m_operatorController
    // .getYButton()
    // .whileTrue(new InstantCommand(
    // () -> m_armSubsystem.toPos(Rotation2d.fromRadians(Math.PI / 2))));

    // Menglins code for keyboard simulation
    // // Arm Up
    // new JoystickButton(m_operatorController.getHID(), 1)
    // .whileTrue(new StartEndCommand(() -> m_armSubsystem.up(), () ->
    // m_armSubsystem.stop()));

    // // Arm Down
    // new JoystickButton(m_operatorController.getHID(), 2)
    // .whileTrue(new StartEndCommand(() -> m_armSubsystem.down(), () ->
    // m_armSubsystem.stop()));

    // // Arm Pos
    // new JoystickButton(m_operatorController.getHID(), 3)
    // .whileTrue(new InstantCommand(() -> m_armSubsystem.pos()));

    // Arm Up
    m_operatorController
        .getAButton()
        .whileTrue(new StartEndCommand(() -> m_wristSubsystem.up(), () -> m_wristSubsystem.stop()));

    // Arm down
    m_operatorController
        .getBButton()
        .whileTrue(
            new StartEndCommand(() -> m_wristSubsystem.down(), () -> m_wristSubsystem.stop()));

    m_operatorController
        .getLeftBumper()
        .whileTrue(
            new StartEndCommand(() -> m_armSubsystem.up(), () -> m_armSubsystem.stop()));

    // DOWN
    m_operatorController
        .getRightBumper()
        .whileTrue(
            new StartEndCommand(() -> m_armSubsystem.down(), () -> m_armSubsystem.stop()));

    // Arm Pos
    m_operatorController
        .getXButton()
        .whileTrue(new InstantCommand(() -> m_wristSubsystem.pos(Constants.kWristStow)));

    // Wrist Pos Amp
    m_operatorController
        .getYButton()
        .whileTrue(new InstantCommand(() -> m_wristSubsystem.pos(Constants.kWristAmp)));

    m_operatorController
        .getStartButton()
        .whileTrue(new InstantCommand(() -> m_wristSubsystem.pos(Constants.kWristSubwoofer)));
  }
}
