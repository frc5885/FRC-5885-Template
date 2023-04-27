// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TankConstants;
import frc.robot.subsystems.TankDriveSubsystem.TankDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PerformSPath extends SequentialCommandGroup {
  TankDrive m_tank;
  DifferentialDriveVoltageConstraint autoVoltageConstraint;
  TrajectoryConfig config;
  Trajectory exampleTrajectory;

  /** Creates a new PerformSPath. */
  public PerformSPath(TankDrive tank) {

    m_tank = tank;
    // autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             TankConstants.Auto.ksVolts,
    //             TankConstants.Auto.kvVoltSecondsPerMeter,
    //             TankConstants.Auto.kaVoltSecondsSquaredPerMeter),
    //         TankConstants.Auto.kDriveKinematics,
    //         10);

    // config =
    //     new TrajectoryConfig(
    //             TankConstants.Auto.kMaxSpeedMetersPerSecond,
    //             TankConstants.Auto.kMaxAccelerationMetersPerSecondSquared)
    //         .setKinematics(TankConstants.Auto.kDriveKinematics)
    //         .addConstraint(autoVoltageConstraint);

    // exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //         new Pose2d(3, 0, new Rotation2d(0)),
    //         config);

    PathPlannerTrajectory examplePath =
        PathPlanner.loadPath(
            "New Path Copy",
            new PathConstraints(
                TankConstants.Auto.kMaxSpeedMetersPerSecond,
                TankConstants.Auto.kMaxAccelerationMetersPerSecondSquared));

    addCommands(followTrajectoryCommand(examplePath, true));
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                m_tank.resetOdometry(traj.getInitialPose());
              }
            }),
        new PPRamseteCommand(
            traj,
            m_tank::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(
                TankConstants.Auto.ksVolts,
                TankConstants.Auto.kvVoltSecondsPerMeter,
                TankConstants.Auto.kaVoltSecondsSquaredPerMeter),
            TankConstants.Auto.kDriveKinematics, // DifferentialDriveKinematics
            m_tank::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(
                TankConstants.Auto.kPDriveVel,
                0,
                0), // Left controller. Tune these values for your robot. Leaving them 0 will only
            // use feedforwards.
            new PIDController(
                TankConstants.Auto.kPDriveVel,
                0,
                0), // Right controller (usually the same values as left controller)
            m_tank::driveTankVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color.
            // Optional, defaults to true
            m_tank // Requires this drive subsystem
            ));
  }

  // public Command getSPathCommand(Tank tank) {

  //   RamseteCommand ramseteCommand =
  //       new RamseteCommand(
  //           exampleTrajectory,
  //           m_tank::getPose,
  //           new RamseteController(TankConstants.Auto.kRamseteB, TankConstants.Auto.kRamseteZeta),
  //           new SimpleMotorFeedforward(
  //               TankConstants.Auto.ksVolts,
  //               TankConstants.Auto.kvVoltSecondsPerMeter,
  //               TankConstants.Auto.kaVoltSecondsSquaredPerMeter),
  //           TankConstants.Auto.kDriveKinematics,
  //           m_tank::getWheelSpeeds,
  //           new PIDController(TankConstants.Auto.kPDriveVel, 0, 0),
  //           new PIDController(TankConstants.Auto.kPDriveVel, 0, 0),
  //           m_tank::driveTankVolts,
  //           m_tank);

  //   return ramseteCommand.andThen(
  //       () -> {
  //         System.out.println("Done!");
  //         m_tank.driveTankVolts(0, 0);
  //       });
  // }
}
