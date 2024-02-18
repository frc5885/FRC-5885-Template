package frc.robot;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.base.WCRobot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.io.DriverController;
import frc.robot.base.io.OperatorController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class Robot extends WCRobot {

  Beambreak m_beambreak;
  ArmSubsystem m_armSubsystem;
  WristSubsystem m_wristSubsystem;
  FeederSubsystem m_feederSubsystem;
  IntakeSubsystem m_intakeSubsystem;

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
  }

  @Override
  protected void initDriverControllerBindings(DriverController m_driverController) {

    // Feeder
    m_driverController.getAButton()
        .whileTrue(
            new StartEndCommand(
                () -> m_feederSubsystem.intake(),
                () -> m_feederSubsystem.stop()
            )
        );

    // Arm - Up
    m_driverController.getRightBumper()
        .whileTrue(
            new StartEndCommand(
                () -> m_armSubsystem.up(),
                () -> m_armSubsystem.stop()
            )
        );

    // Arm - Down
    m_driverController.getLeftBumper()
        .whileTrue(
            new StartEndCommand(
                () -> m_armSubsystem.down(),
                () -> m_armSubsystem.stop()
            )
        );

    // Wrist - Forward
    m_driverController.getStartButton()
        .whileTrue(
            new StartEndCommand(
                () -> m_wristSubsystem.forward(),
                () -> m_wristSubsystem.stop()
            )
        );

    // Wrist - Reverse
    m_driverController.getBackButton()
        .whileTrue(
            new StartEndCommand(
                () -> m_wristSubsystem.reverse(),
                () -> m_wristSubsystem.stop()
            )
        );
  }

  @Override
  protected void initOperatorControllerBindings(OperatorController m_operatorController) {

  }
}
