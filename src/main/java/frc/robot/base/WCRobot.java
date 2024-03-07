// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.base;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.base.io.DriverController;
import frc.robot.base.io.OperatorController;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.base.subsystems.WCPathPlanner;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.commands.SwerveJoystickCmd;
import java.util.Optional;

public abstract class WCRobot {

  // Controller
  final DriverController m_driverController;
  final OperatorController m_operatorController;

  private final SwerveDriveSubsystem m_swerveDrive;
  private final SwervePoseEstimator m_swervePoseEstimator;
  private final PhotonVisionSystem m_photonVision;
  private final WCPathPlanner m_simplePathPlanner;

  boolean m_isFieldOriented = true;

  boolean m_isAimbotting = false;

  public WCRobot() {
    m_swerveDrive = new SwerveDriveSubsystem();
    m_photonVision = new PhotonVisionSystem();
    m_swervePoseEstimator = new SwervePoseEstimator(m_swerveDrive, m_photonVision);
    m_simplePathPlanner = new WCPathPlanner(m_swervePoseEstimator, m_swerveDrive);
    m_operatorController = new OperatorController();
    m_driverController =
        new DriverController(
            new InstantCommand(
                () -> {
                  m_swerveDrive.resetGyro();
                  m_swervePoseEstimator.reset();
                }),
            new InstantCommand(() -> m_isFieldOriented = !m_isFieldOriented));

    initComponents();
    initSubsystems();
    initAutoCommands();
    initDriverControllerBindings(m_driverController);
    initOperatorControllerBindings(m_operatorController);
    initSwerveBindings();

    // PATHPLANNER AUTO STUFF (building the auto chooser has to be done after named commands are
    // registered in initAutoCommands())
    m_simplePathPlanner.setRotationTargetOverrideFunction(this::getOverrideAutoTargetRotation);
    m_simplePathPlanner.buildAutoChooser();

    SmartDashboard.putBoolean("Aimbotting", m_isAimbotting);
  }

  private void initSwerveBindings() {
    m_swerveDrive.setDefaultCommand(
        new SwerveJoystickCmd(
            m_swerveDrive,
            m_swervePoseEstimator,
            m_photonVision,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -getDriverRotationAxis(),
            () -> m_isFieldOriented,
            () -> m_isAimbotting));
  }

  // so that aimBotting can be set and accessed by other stuff in Robot.java
  protected Boolean isAimBotting() {
    return m_isAimbotting;
  }

  public void setAimBotting(Boolean value) {
    m_isAimbotting = value;
    SmartDashboard.putBoolean("Aimbotting", m_isAimbotting);
  }

  protected double getDriverRotationAxis() {
    // if the driver is trying to rotate, turn off aimbotting
    double position = m_driverController.getRightX();
    if (Math.abs(position) > 0.1) {
      setAimBotting(false);
    }
    return m_driverController.getRightX();
  }

  // for pathplanner autos
  protected Optional<Rotation2d> getOverrideAutoTargetRotation() {
    // if aimbotting is on, return the angle to the target
    if (m_isAimbotting) {
      double angleToTarget =
          m_photonVision.getAngleToTarget(
              m_swervePoseEstimator.getPose(), m_photonVision.getTargetID());
      return Optional.of(Rotation2d.fromRadians(angleToTarget));
    }
    // otherwise, return empty and pathplanner will use the orientation from the path
    return Optional.empty();
  }

  protected void pathPlannerRegisterNamedCommand(String name, Command command) {
    m_simplePathPlanner.registerNamedCommand(name, command);
  }

  protected abstract void initComponents();

  protected abstract void initSubsystems();

  protected abstract void initAutoCommands();

  protected abstract void initDriverControllerBindings(DriverController m_driverController);

  protected abstract void initOperatorControllerBindings(OperatorController m_operatorController);

  protected Command getAutonomousCommand() {
    // return new SimplePathPlanner(m_swervePoseEstimator, m_swerveDrive);
    return m_simplePathPlanner.getSelectedAuto();
  }
}
