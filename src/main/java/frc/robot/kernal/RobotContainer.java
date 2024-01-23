// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kernal;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TuningCommands.SwerveGetModuleOffsets;
import frc.robot.commands.TuningCommands.SwerveSolveFeedForward;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleNEO;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Controller
  private final CommandXboxController m_driverController;
  // private final CommandXboxController m_operatorController;

  private final SwerveDrive m_swerveDrive;
  private final SwervePoseEstimator m_swervePoseEstimator;

  private final LoggedDashboardChooser<Command> m_autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  public RobotContainer() {
    // Setup controllers depending on the current mode
    switch (Constants.kCurrentMode) {
      case REAL:
        if (!RobotBase.isReal()) {
          DriverStation.reportError("Attempted to run REAL on SIMULATED robot!", false);
          throw new NoSuchMethodError("Attempted to run REAL on SIMULATED robot!");
        }
        m_swerveDrive =
            new SwerveDrive(
                new SwerveModuleNEO(
                    ModuleConstants.kLeftFrontDriveMotorID,
                    ModuleConstants.kLeftFrontTurnMotorID,
                    ModuleConstants.kLeftFrontAnalogEncoderPort,
                    ModuleConstants.kLeftFrontModuleOffset,
                    ModuleConstants.kLeftFrontTurnMotorInverted,
                    ModuleConstants.kLeftFrontDriveMotorInverted),
                new SwerveModuleNEO(
                    ModuleConstants.kRightFrontDriveMotorID,
                    ModuleConstants.kRightFrontTurnMotorID,
                    ModuleConstants.kRightFrontAnalogEncoderPort,
                    ModuleConstants.kRightFrontModuleOffset,
                    ModuleConstants.kRightFrontTurnMotorInverted,
                    ModuleConstants.kRightFrontDriveMotorInverted),
                new SwerveModuleNEO(
                    ModuleConstants.kLeftRearDriveMotorID,
                    ModuleConstants.kLeftRearTurnMotorID,
                    ModuleConstants.kLeftRearAnalogEncoderPort,
                    ModuleConstants.kLeftRearModuleOffset,
                    ModuleConstants.kLeftRearTurnMotorInverted,
                    ModuleConstants.kLeftRearDriveMotorInverted),
                new SwerveModuleNEO(
                    ModuleConstants.kRightRearDriveMotorID,
                    ModuleConstants.kRightRearTurnMotorID,
                    ModuleConstants.kRightRearAnalogEncoderPort,
                    ModuleConstants.kRightRearModuleOffset,
                    ModuleConstants.kRightRearTurnMotorInverted,
                    ModuleConstants.kRightRearDriveMotorInverted));
        break;

      case SIMULATOR:
        if (RobotBase.isReal()) {
          DriverStation.reportError("Attempted to run SIMULATED on REAL robot!", false);
          throw new NoSuchMethodError("Attempted to run SIMULATED on REAL robot!");
        }
        m_swerveDrive =
            new SwerveDrive(
                new SwerveModuleSim(false),
                new SwerveModuleSim(false),
                new SwerveModuleSim(false),
                new SwerveModuleSim(false));

        break;

      default:
        throw new NoSuchMethodError("Not Implemented");
    }

    m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);
    // m_operatorController = new
    // CommandXboxController(ControllerConstants.kOperatorControllerPort);

    m_swervePoseEstimator = new SwervePoseEstimator(m_swerveDrive);
    m_swervePoseEstimator.reset(new Pose2d(0, 0, new Rotation2d()));

    AutoBuilder.configureHolonomic(
        m_swervePoseEstimator::getPose, // Robot pose supplier
        m_swervePoseEstimator
            ::reset, // Method to reset odometry (will be called if your auto has a starting pose)
        m_swerveDrive::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        m_swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            3.5, // Max module speed, in m/s
            0.53, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
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
        m_swerveDrive // Reference to this subsystem to set requirements
        );

    configureBindings();
  }

  boolean m_isFieldOriented = true;

  private void configureBindings() {

    m_swerveDrive.setDefaultCommand(
        new SwerveJoystickCmd(
            m_swerveDrive,
            m_swervePoseEstimator,
            () -> (-m_driverController.getLeftY()),
            () -> (-m_driverController.getLeftX()),
            () -> (-m_driverController.getRightX()),
            () -> (m_isFieldOriented)));

    // m_swerveDrive.setDefaultCommand(new SwerveGetModuleOffsets(m_swerveDrive));

    new JoystickButton(m_driverController.getHID(), Button.kX.value)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> {
                      m_swerveDrive.resetGyro();
                      m_swervePoseEstimator.reset(new Pose2d(0, 0, new Rotation2d()));
                    })));

    new JoystickButton(m_driverController.getHID(), Button.kY.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_isFieldOriented = !m_isFieldOriented;
                }));

    // new JoystickButton(m_driverController.getHID(), Button.kB.value)
    //     .whileTrue(new SimplePathPlanner(m_swervePoseEstimator));

    m_autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    m_autoChooser.addOption(
        "[TUNING] Get Module Offsets", new SwerveGetModuleOffsets(m_swerveDrive));
    m_autoChooser.addOption(
        "[TUNING] Get Swerve FF Characteristics", new SwerveSolveFeedForward(m_swerveDrive));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}
