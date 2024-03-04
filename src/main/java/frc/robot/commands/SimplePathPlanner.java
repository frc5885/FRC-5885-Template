// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;

public class SimplePathPlanner extends SequentialCommandGroup {

  SwervePoseEstimator m_poseEstimator;
  SwerveDriveSubsystem m_robotDrive;

  /** Creates a new SimplePathPlanner. */
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
      robotDrive
    );
  }
}
