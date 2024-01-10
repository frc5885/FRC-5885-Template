// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kernal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TuningCommands.SwerveGetModuleOffsets;
import frc.robot.commands.TuningCommands.SwerveSolveFeedForward;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleNEO;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Controller
  private final CommandXboxController m_driverController;
  // private final CommandXboxController m_operatorController;

  private final SwerveDrive m_swerveDrive;
  private final SwervePoseEstimator m_swervePoseEstimator;

  private final LoggedDashboardChooser<Command> m_autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  public RobotContainer() {
    // Setup controllers depending on the current mode
    switch (Constants.kCurrentMode) {
      case REAL:
        if (!RobotBase.isReal()) {
          DriverStation.reportError("Attempted to run REAL on SIMULATED robot!", false);
          throw new NoSuchMethodError("Attempted to run REAL on SIMULATED robot!");
        }
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
        break;

      case SIMULATOR:
        if (RobotBase.isReal()) {
          DriverStation.reportError("Attempted to run SIMULATED on REAL robot!", false);
          throw new NoSuchMethodError("Attempted to run SIMULATED on REAL robot!");
        }
        m_swerveDrive =
            new SwerveDrive(
                new SwerveModuleSim(false),
                new SwerveModuleSim(false),
                new SwerveModuleSim(false),
                new SwerveModuleSim(false));

        break;

      default:
        throw new NoSuchMethodError("Not Implemented");
    }

    m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);
    // m_operatorController = new
    // CommandXboxController(ControllerConstants.kOperatorControllerPort);

    m_swervePoseEstimator = new SwervePoseEstimator(m_swerveDrive);
    m_swervePoseEstimator.reset(new Pose2d(0, 0, new Rotation2d()));

    configureBindings();
  }

  boolean m_isFieldOriented = true;

  private void configureBindings() {

    m_swerveDrive.setDefaultCommand(
        new SwerveJoystickCmd(
            m_swerveDrive,
            m_swervePoseEstimator,
            () -> (-m_driverController.getLeftY()),
            () -> (-m_driverController.getLeftX()),
            () -> (-m_driverController.getRightX()),
            () -> (m_isFieldOriented)));

    // m_swerveDrive.setDefaultCommand(new SwerveGetModuleOffsets(m_swerveDrive));

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

    m_autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    m_autoChooser.addOption(
        "[TUNING] Get Module Offsets", new SwerveGetModuleOffsets(m_swerveDrive));
    m_autoChooser.addOption(
        "[TUNING] Get Swerve FF Characteristics", new SwerveSolveFeedForward(m_swerveDrive));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}
