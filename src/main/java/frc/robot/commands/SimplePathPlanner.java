// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import java.util.Optional;
import java.util.function.Supplier;

public class SimplePathPlanner extends SequentialCommandGroup {

  SwervePoseEstimator m_poseEstimator;
  SwerveDriveSubsystem m_robotDrive;
  SendableChooser<Command> m_autoChooser;

  // creates a new pathplanner autobuilder
  public SimplePathPlanner(SwervePoseEstimator poseEstimator, SwerveDriveSubsystem robotDrive) {

    m_poseEstimator = poseEstimator;
    m_robotDrive = robotDrive;

    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
        m_poseEstimator::getPose,
        m_poseEstimator::resetPose,
        m_robotDrive::getChassisSpeeds,
        m_robotDrive::setChassisSpeeds,
        AutoConstants.pathFollowerConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        robotDrive);
  }

  public void setRotationTargetOverrideFunction(
      Supplier<Optional<Rotation2d>> targetRotationOverrideFunction) {
    // this takes a function that returns an optional, if it is present, the target angle from the
    // auto will be overridden (for better aiming)
    PPHolonomicDriveController.setRotationTargetOverride(targetRotationOverrideFunction);
  }

  public void buildAutoChooserAndPutOnSmartDash() {
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Select", m_autoChooser);
  }

  public Command getSelectedAuto() {
    return m_autoChooser.getSelected();
  }

  public void registerNamedCommand(String name, Command command) {
    NamedCommands.registerCommand(name, command);
  }
}
