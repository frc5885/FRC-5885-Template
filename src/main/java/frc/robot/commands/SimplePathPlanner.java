// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimplePathPlanner extends SequentialCommandGroup {

  ChoreoTrajectory traj;

  /** Creates a new SimplePathPlanner. */
  public SimplePathPlanner(SwervePoseEstimator poseEstimator, SwerveDriveSubsystem m_robotDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    traj = Choreo.getTrajectory("path close 1");

    var thetaController = new PIDController(1, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addCommands(
        new InstantCommand(
            () -> {
              poseEstimator.reset(traj.getInitialPose());
            }),
        Choreo.choreoSwerveCommand(
            traj, // Choreo trajectory from above
            poseEstimator
                ::getPose, // A function that returns the current field-relative pose of the robot:
            // your
            // wheel or vision odometry
            new PIDController(1, 0.0, 0.0), // PIDController for field-relative X
            // translation (input: X error in meters,
            // output: m/s).
            new PIDController(1, 0.0, 0.0), // PIDController for field-relative Y
            // translation (input: Y error in meters,
            // output: m/s).
            thetaController, // PID constants to correct for rotation
            // error
            m_robotDrive::setChassisSpeeds,
            () ->
                false, // Whether or not to mirror the path based on alliance (this assumes the path
            // is
            // created for the blue alliance)
            m_robotDrive // The subsystem(s) to require, typically your drive subsystem only
            ));
    // addCommands(
    // new InstantCommand(
    // () -> {
    // poseEstimator.reset(traj.getInitialPose());
    // }),
    // Choreo.choreoSwerveCommand(
    // traj, // Choreo trajectory from above
    // poseEstimator
    // ::getPose, // A function that returns the current field-relative pose of the
    // robot:
    // // your
    // // wheel or vision odometry
    // new PIDController(1, 0.0, 0.0), // PIDController for field-relative X
    // // translation (input: X error in meters,
    // // output: m/s).
    // new PIDController(1, 0.0, 0.0), // PIDController for field-relative Y
    // // translation (input: Y error in meters,
    // // output: m/s).
    // thetaController, // PID constants to correct for rotation
    // // error
    // m_robotDrive::setChassisSpeeds,
    // () -> {
    // return DriverStation.getAlliance()
    // .orElse(DriverStation.Alliance.Blue)
    // .equals(DriverStation.Alliance.Red);
    // }, // Whether or not to mirror the path based on alliance (this assumes the
    // path is
    // // created for the blue alliance)
    // m_robotDrive // The subsystem(s) to require, typically your drive subsystem
    // only
    // ));
  }
}
