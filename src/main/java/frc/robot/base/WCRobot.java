// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.base.io.DriverController;
import frc.robot.base.io.OperatorController;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.commands.SimplePathPlanner;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.PoseEstimatorSubsystem.PhotonVisionSystem;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;

public abstract class WCRobot {

  // Controller
  final DriverController m_driverController;
  final OperatorController m_operatorController;

  private final SwerveDriveSubsystem m_swerveDrive;
  private final SwervePoseEstimator m_swervePoseEstimator;
  private final PhotonVisionSystem m_photonVision;

  boolean m_isFieldOriented = true;

  boolean m_isAimbotting = false;

  public WCRobot() {
    m_swerveDrive = new SwerveDriveSubsystem();
    m_photonVision = new PhotonVisionSystem();
    m_swervePoseEstimator = new SwervePoseEstimator(m_swerveDrive, m_photonVision);
    m_operatorController = new OperatorController();
    m_driverController =
        new DriverController(
            new InstantCommand(
                () -> {
                  m_swerveDrive.resetGyro();
                  m_swervePoseEstimator.reset();
                }),
            new InstantCommand(() -> m_isFieldOriented = !m_isFieldOriented),
            // new StartEndCommand(() -> m_isAimbotting = true, () -> m_isAimbotting = false));
            new InstantCommand(() -> m_isAimbotting = !m_isAimbotting));

    initComponents();
    initSubsystems();
    initDriverControllerBindings(m_driverController);
    initOperatorControllerBindings(m_operatorController);
    initSwerveBindings();
  }

  private void initSwerveBindings() {
    m_swerveDrive.setDefaultCommand(
        new SwerveJoystickCmd(
            m_swerveDrive,
            m_swervePoseEstimator,
            m_photonVision,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX(),
            () -> m_isFieldOriented,
            () -> m_isAimbotting));
  }

  protected abstract void initComponents();

  protected abstract void initSubsystems();

  protected abstract void initDriverControllerBindings(DriverController m_driverController);

  protected abstract void initOperatorControllerBindings(OperatorController m_operatorController);

  protected Command getAutonomousCommand() {
    return new SimplePathPlanner(m_swervePoseEstimator, m_swerveDrive);
  }
  ;
}
