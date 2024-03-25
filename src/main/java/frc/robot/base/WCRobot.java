// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.base;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.WCLogger;
import frc.robot.base.io.DriverController;
import frc.robot.base.io.OperatorController;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.base.subsystems.WCPathPlanner;
import frc.robot.base.subsystems.swerve.SwerveAction;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.commands.SwerveJoystickCommand;
import java.util.Optional;

public abstract class WCRobot {

  // Controller
  final DriverController m_driverController;
  final OperatorController m_operatorController;

  public final SwerveDriveSubsystem m_swerveDrive;
  public final SwervePoseEstimator m_swervePoseEstimator;
  public final PhotonVisionSystem m_photonVision;
  public final WCPathPlanner m_simplePathPlanner;

  protected boolean m_isFieldOriented = true;

  protected SwerveAction m_swerveAction = SwerveAction.DEFAULT;

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // protected boolean m_isAimbotting = false;

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
                  // m_swerveDrive.resetGyro();
                  // m_swervePoseEstimator.reset();
                }),
            new InstantCommand(
                () -> {
                  m_isFieldOriented = !m_isFieldOriented;
                }));

    initComponents();
    initSubsystems();
    initAutoCommands();
    initDriverControllerBindings(m_driverController);
    initOperatorControllerBindings(m_operatorController);
    initSwerveBindings();

    // PATHPLANNER AUTO STUFF (building the auto chooser has to be done after named
    // commands are
    // registered in initAutoCommands())
    m_simplePathPlanner.setRotationTargetOverrideFunction(this::getOverrideAutoTargetRotation);
    m_simplePathPlanner.buildAutoChooser();

    // SmartDashboard.putBoolean("Aimbotting", m_isAimbotting);
  }

  private void initSwerveBindings() {
    m_swerveDrive.setDefaultCommand(
        new SwerveJoystickCommand(
            m_swerveDrive,
            m_swervePoseEstimator,
            m_photonVision,
            m_driverController::getLeftY,
            () -> -m_driverController.getLeftX(),
            () -> -getDriverRotationAxis(),
            () -> m_isFieldOriented,
            // () -> isAimBotting(),
            this::getSwerveAction));
  }

  // so that aimBotting can be set and accessed by other stuff in Robot.java
  // protected Boolean isAimBotting() {
  // return m_isAimbotting;
  // }

  public SwerveAction getSwerveAction() {
    return m_swerveAction;
  }

  // public void setAimBotting(Boolean value) {
  // m_isAimbotting = value;
  // SmartDashboard.putBoolean("Aimbotting", m_isAimbotting);
  // }

  public void setSwerveAction(SwerveAction desiredAction) {
    m_swerveAction = desiredAction;
    WCLogger.putString(this, "SwerveAction", m_swerveAction != null ? m_swerveAction.toString() : "null");
  }

  protected double getDriverRotationAxis() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // if the driver is trying to rotate, turn off aimbotting
    double rightPosition = m_driverController.getRightX();
    double robotHeadingDeg = m_swervePoseEstimator.getPose().getRotation().getDegrees();
    if (Math.abs(rightPosition) > 0.3
        || (m_swerveAction == SwerveAction.FACEFORWARD
            && MathUtil.isNear(alliance == Alliance.Blue ? 0 : 180, robotHeadingDeg, 2.5))
        || (m_swerveAction == SwerveAction.FACEBACKWARD
            && MathUtil.isNear(alliance == Alliance.Blue ? 180 : 0, robotHeadingDeg, 2.5))
        || (m_swerveAction == SwerveAction.FACEAMP
            && MathUtil.isNear(alliance == Alliance.Blue ? 90 : 90, robotHeadingDeg, 2.5))) {
      // setAimBotting(false);
      setSwerveAction(SwerveAction.DEFAULT);
    }
    return m_driverController.getRightX();
  }

  // for pathplanner autos
  protected Optional<Rotation2d> getOverrideAutoTargetRotation() {
    // if aimbotting is on, return the angle to the target
    if (m_swerveAction == SwerveAction.AIMBOTTING) {
      double angleToTarget =
          m_photonVision.getAngleToTarget(
              m_swervePoseEstimator.getPose(), m_photonVision.getTargetID());
      return Optional.of(Rotation2d.fromRadians(angleToTarget));
    }
    // otherwise, return empty and pathplanner will use the orientation from the
    // path
    return Optional.empty();
  }

  protected void pathPlannerRegisterNamedCommand(String name, Command command) {
    m_simplePathPlanner.registerNamedCommand(name, command);
  }

  protected abstract void initComponents();

  protected abstract void initSubsystems();

  protected abstract void initAutoCommands()
      // {
      //   m_chooser.setDefaultOption("Do Nothing", new InstantCommand());
      //   m_chooser.addOption(
      //       "[TUNING] SysID Quasistatic Forward",
      //       m_swerveDrive.getSysIdQuasistatic(Direction.kForward));
      //   m_chooser.addOption(
      //       "[TUNING] SysID Quasistatic Backwards",
      //       m_swerveDrive.getSysIdQuasistatic(Direction.kReverse));
      //   m_chooser.addOption(
      //       "[TUNING] SysID Dynamic Forward", m_swerveDrive.getSysIdDynamic(Direction.kForward));
      //   m_chooser.addOption(
      //       "[TUNING] SysID Dynamic Backwards",
      // m_swerveDrive.getSysIdDynamic(Direction.kReverse));

      //   SmartDashboard.putData("Auto Choices", m_chooser);
      // }
      ;

  protected abstract void initDriverControllerBindings(DriverController m_driverController);

  protected abstract void initOperatorControllerBindings(OperatorController m_operatorController);

  protected Command getAutonomousCommand() {
    // return new SimplePathPlanner(m_swervePoseEstimator, m_swerveDrive);
    return m_simplePathPlanner.getSelectedAuto();
    // return m_chooser.getSelected();
  }
}
