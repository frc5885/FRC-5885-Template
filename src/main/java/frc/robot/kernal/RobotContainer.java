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
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleNEO;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleSim;

public class RobotContainer {

  // Controller
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final SwerveDrive m_swerveDrive;
  private final SwervePoseEstimator m_swervePoseEstimator;

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
                    SwerveConstants.kLeftFrontDriveMotorID,
                    SwerveConstants.kLeftFrontTurnMotorID,
                    SwerveConstants.kLeftFrontAnalogEncoderPort,
                    SwerveConstants.kLeftFrontModuleOffset,
                    SwerveConstants.kLeftFrontTurnMotorInverted,
                    SwerveConstants.kLeftFrontDriveMotorInverted),
                new SwerveModuleNEO(
                    SwerveConstants.kRightFrontDriveMotorID,
                    SwerveConstants.kRightFrontTurnMotorID,
                    SwerveConstants.kRightFrontAnalogEncoderPort,
                    SwerveConstants.kRightFrontModuleOffset,
                    SwerveConstants.kRightFrontTurnMotorInverted,
                    SwerveConstants.kRightFrontDriveMotorInverted),
                new SwerveModuleNEO(
                    SwerveConstants.kLeftRearDriveMotorID,
                    SwerveConstants.kLeftRearTurnMotorID,
                    SwerveConstants.kLeftRearAnalogEncoderPort,
                    SwerveConstants.kLeftRearModuleOffset,
                    SwerveConstants.kLeftRearTurnMotorInverted,
                    SwerveConstants.kLeftRearDriveMotorInverted),
                new SwerveModuleNEO(
                    SwerveConstants.kRightRearDriveMotorID,
                    SwerveConstants.kRightRearTurnMotorID,
                    SwerveConstants.kRightRearAnalogEncoderPort,
                    SwerveConstants.kRightRearModuleOffset,
                    SwerveConstants.kRightRearTurnMotorInverted,
                    SwerveConstants.kRightRearDriveMotorInverted));
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
    m_operatorController = new CommandXboxController(ControllerConstants.kOperatorControllerPort);

    m_swervePoseEstimator = new SwervePoseEstimator(m_swerveDrive);

    configureBindings();
  }

  private void configureBindings() {

    m_swerveDrive.setDefaultCommand(
        new SwerveJoystickCmd(
            m_swerveDrive,
            m_swervePoseEstimator,
            () -> (-m_driverController.getLeftY()),
            () -> (-m_driverController.getLeftX()),
            () -> (-m_driverController.getRightX()),
            () -> (true)));

    new JoystickButton(m_driverController.getHID(), Button.kX.value)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> {
                      m_swerveDrive.resetGyro();
                      m_swervePoseEstimator.reset(new Pose2d(0, 0, new Rotation2d()));
                    })));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
