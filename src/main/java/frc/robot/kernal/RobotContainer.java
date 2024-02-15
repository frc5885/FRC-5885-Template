// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kernal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TuningCommands.SwerveGetModuleOffsets;
import frc.robot.commands.TuningCommands.SwerveSolveFeedForward;
import frc.robot.components.Beambreak;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleNEO;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleSim;
import frc.robot.subsystems.WristSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Controller
  private final CommandXboxController m_driverController;
  // private final CommandXboxController m_operatorController;

  private final SwerveDrive m_swerveDrive;
  private final SwervePoseEstimator m_swervePoseEstimator;

  private final LoggedDashboardChooser<Command> m_autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private final LoggedDashboardChooser<Command> m_initialPoseChooser =
      new LoggedDashboardChooser<>("Starting Pose");

  private final Beambreak m_beambreak = new Beambreak();

  public RobotContainer() {
    // Setup controllers depending on the current mode
    if (RobotBase.isReal()) {
      m_swerveDrive =
          new SwerveDrive(
              new SwerveModuleNEO(
                  ModuleConstants.kLeftFrontDriveMotorID,
                  ModuleConstants.kLeftFrontTurnMotorID,
                  ModuleConstants.kLeftFrontAnalogEncoderPort,
                  ModuleConstants.kLeftFrontModuleOffset,
                  ModuleConstants.kLeftFrontTurnMotorInverted,
                  ModuleConstants.kLeftFrontDriveMotorInverted),
              new SwerveModuleNEO(
                  ModuleConstants.kRightFrontDriveMotorID,
                  ModuleConstants.kRightFrontTurnMotorID,
                  ModuleConstants.kRightFrontAnalogEncoderPort,
                  ModuleConstants.kRightFrontModuleOffset,
                  ModuleConstants.kRightFrontTurnMotorInverted,
                  ModuleConstants.kRightFrontDriveMotorInverted),
              new SwerveModuleNEO(
                  ModuleConstants.kLeftRearDriveMotorID,
                  ModuleConstants.kLeftRearTurnMotorID,
                  ModuleConstants.kLeftRearAnalogEncoderPort,
                  ModuleConstants.kLeftRearModuleOffset,
                  ModuleConstants.kLeftRearTurnMotorInverted,
                  ModuleConstants.kLeftRearDriveMotorInverted),
              new SwerveModuleNEO(
                  ModuleConstants.kRightRearDriveMotorID,
                  ModuleConstants.kRightRearTurnMotorID,
                  ModuleConstants.kRightRearAnalogEncoderPort,
                  ModuleConstants.kRightRearModuleOffset,
                  ModuleConstants.kRightRearTurnMotorInverted,
                  ModuleConstants.kRightRearDriveMotorInverted));
    } else {
      // Running in a simulator
      m_swerveDrive =
          new SwerveDrive(
              new SwerveModuleSim(false),
              new SwerveModuleSim(false),
              new SwerveModuleSim(false),
              new SwerveModuleSim(false));
    }

    m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);
    // m_operatorController = new
    // CommandXboxController(ControllerConstants.kOperatorControllerPort);

    m_swervePoseEstimator = new SwervePoseEstimator(m_swerveDrive);
    m_swervePoseEstimator.reset(new Pose2d(0, 0, new Rotation2d()));

    m_autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    m_autoChooser.addOption(
        "[TUNING] Get Module Offsets", new SwerveGetModuleOffsets(m_swerveDrive));
    m_autoChooser.addOption(
        "[TUNING] Get Swerve FF Characteristics", new SwerveSolveFeedForward(m_swerveDrive));
    m_autoChooser.addOption(
        "[TUNING] SysID Quasistatic Forward",
        m_swerveDrive.getSysIdQuasistatic(Direction.kForward));
    m_autoChooser.addOption(
        "[TUNING] SysID Quasistatic Backwards",
        m_swerveDrive.getSysIdQuasistatic(Direction.kReverse));
    m_autoChooser.addOption(
        "[TUNING] SysID Dynamic Forward", m_swerveDrive.getSysIdDynamic(Direction.kForward));
    m_autoChooser.addOption(
        "[TUNING] SysID Dynamic Backwards", m_swerveDrive.getSysIdDynamic(Direction.kReverse));

    // m_initialPoseChooser.addDefaultOption(
    //         "Left OFF of Subwoofer",
    //         new InstantCommand(
    //                 () -> {
    //                     m_swervePoseEstimator.reset(AutoStartingPositions.kLeftOffSubwoofer);
    //                 },
    //                 m_swervePoseEstimator));
    // m_initialPoseChooser.addOption(
    //         "Left ON of Subwoofer",
    //         new InstantCommand(
    //                 () -> {
    //                     m_swervePoseEstimator.reset(AutoStartingPositions.kLeftOnSubwoofer);
    //                 },
    //                 m_swervePoseEstimator));

    configureBindings();
  }

  boolean m_isFieldOriented = true;

  private void configureBindings() {

    // IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(m_beambreak);

    ArmSubsystem m_armSubsystem = new ArmSubsystem();

    WristSubsystem m_wristSubsystem = new WristSubsystem();

    FeederSubsystem m_feederSubsystem = new FeederSubsystem(m_beambreak);
    // m_initialPoseChooser.addDefaultOption(
    //         "Left OFF of Subwoofer",
    //         new InstantCommand(
    //         () -> {
    //         m_swervePoseEstimator.reset(AutoStartingPositions.kLeftOffSubwoofer);
    //         },
    //         m_swervePoseEstimator));
    // m_initialPoseChooser.addOption(
    //         "Left ON of Subwoofer",
    //         new InstantCommand(
    //         () -> {
    //         m_swervePoseEstimator.reset(AutoStartingPositions.kLeftOnSubwoofer);
    //         },
    //         m_swervePoseEstimator));

    m_swerveDrive.setDefaultCommand(
        new SwerveJoystickCmd(
            m_swerveDrive,
            m_swervePoseEstimator,
            () -> (-m_driverController.getLeftY()),
            () -> (-m_driverController.getLeftX()),
            () -> (-m_driverController.getRightX()),
            () -> (m_isFieldOriented)));

    new JoystickButton(m_driverController.getHID(), Button.kX.value)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> {
                      m_swerveDrive.resetGyro();
                      m_swervePoseEstimator.reset(new Pose2d(0, 0, new Rotation2d()));
                    })));

    new JoystickButton(m_driverController.getHID(), Button.kY.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_isFieldOriented = !m_isFieldOriented;
                }));

    new JoystickButton(m_driverController.getHID(), Button.kA.value)
        .whileTrue(
            new InstantCommand(
                () -> {
                  m_feederSubsystem.feedShooter(true);
                }))
        .whileFalse(
            new InstantCommand(
                () -> {
                  m_feederSubsystem.feedShooter(false);
                }));

    new JoystickButton(m_driverController.getHID(), Button.kRightBumper.value)
        .whileTrue(
            new InstantCommand(
                () -> {
                  m_armSubsystem.moveArm(false, false);
                }))
        .whileFalse(
            new InstantCommand(
                () -> {
                  m_armSubsystem.moveArm(false, true);
                  ;
                }));
    new JoystickButton(m_driverController.getHID(), Button.kLeftBumper.value)
        .whileTrue(
            new InstantCommand(
                () -> {
                  m_armSubsystem.moveArm(true, false);
                }))
        .whileFalse(
            new InstantCommand(
                () -> {
                  m_armSubsystem.moveArm(true, true);
                }));
    new JoystickButton(m_driverController.getHID(), Button.kStart.value)
        .whileTrue(
            new InstantCommand(
                () -> {
                  m_wristSubsystem.moveWrist(false, false);
                }))
        .whileFalse(
            new InstantCommand(
                () -> {
                  m_wristSubsystem.moveWrist(false, true);
                }));
    new JoystickButton(m_driverController.getHID(), Button.kBack.value)
        .whileTrue(
            new InstantCommand(
                () -> {
                  m_wristSubsystem.moveWrist(true, false);
                  ;
                }))
        .whileFalse(
            new InstantCommand(
                () -> {
                  m_wristSubsystem.moveWrist(true, true);
                }));

    // m_intakeSubsystem.setDefaultCommand(new InstantCommand(() -> {
    // }, m_intakeSubsystem));

    m_feederSubsystem.setDefaultCommand(new InstantCommand(() -> {}, m_feederSubsystem));
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              m_swervePoseEstimator.reset(new Pose2d());
            }),
        new WaitCommand(1),
        m_autoChooser.get());
  }
}
