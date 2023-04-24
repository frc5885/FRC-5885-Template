// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_robotDrive = new DrivetrainSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.DrivetrainConstants.ksVolts,
                Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
                Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DrivetrainConstants.kDriveKinematics,
            8);

    // Create config for trajectory

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.DrivetrainConstants.kMaxSpeedMetersPerSecond,
                Constants.DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared)

            // Add kinematics to ensure max speed is actually obeyed

            .setKinematics(Constants.DrivetrainConstants.kDriveKinematics)

            // Apply the voltage constraint

            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.

    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(

            // Start at the origin facing the +X direction

            new Pose2d(0, 0, new Rotation2d(0)),

            // Pass through these two interior waypoints, making an 's' curve path

            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),

            // End 3 meters straight ahead of where we started, facing forward

            new Pose2d(3, 0, new Rotation2d(0)),

            // Pass config

            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(
                Constants.DrivetrainConstants.kRamseteB,
                Constants.DrivetrainConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.DrivetrainConstants.ksVolts,
                Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
                Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DrivetrainConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0),

            // RamseteCommand passes volts to the callback

            m_robotDrive::tankDriveVolts,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.

    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.

    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
}
