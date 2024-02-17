// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimplePathPlanner extends SequentialCommandGroup {

  /** Creates a new SimplePathPlanner. */
  public SimplePathPlanner(SwervePoseEstimator poseEstimator, SwerveDrive m_robotDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Load a Choreo trajectory as a PathPlannerPath
    PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromPathFile("Example Path");

    addCommands(
        new InstantCommand(
            () -> {
              poseEstimator.reset(exampleChoreoTraj.getPreviewStartingHolonomicPose());
            }),
        new WaitCommand(1),
        new FollowPathHolonomic(
            exampleChoreoTraj,
            poseEstimator::getPose, // Robot pose supplier
            m_robotDrive::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            m_robotDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
            // ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
                // in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                0.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the
                // options here
                ),
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
            m_robotDrive,
            poseEstimator // Reference to this subsystem to set requirements
            ),
        new HaltRobot(m_robotDrive));
  }
}
