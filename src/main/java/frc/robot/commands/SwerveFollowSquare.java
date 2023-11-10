// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveFollowSquare extends SequentialCommandGroup {

  TrajectoryConfig trajectoryConfig =
      new TrajectoryConfig(2, 3).setKinematics(SwerveConstants.kDriveKinematics);

  Trajectory trajectory;

  ProfiledPIDController thetaController =
      new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Math.PI * 1.5, Math.PI));

  /** Creates a new SwerveFollowSquare. */
  public SwerveFollowSquare(SwerveDrive swerveSubsystem, SwervePoseEstimator poseEstimator) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    PIDController xController = new PIDController(0.5, 0, 0);
    PIDController yController = new PIDController(0.5, 0, 0);

    trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(14.513558, 4.424426, new Rotation2d(0)),
            List.of(new Translation2d(14.513558, 3.424426), new Translation2d(13.513558, 3.424426)),
            new Pose2d(13.513558, 4.424426, Rotation2d.fromDegrees(0)),
            trajectoryConfig);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            poseEstimator::getPose,
            SwerveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModuleStates,
            swerveSubsystem);

    addCommands(swerveControllerCommand);
    // new InstantCommand(() -> swerveSubsystem.stop()));
  }
}
