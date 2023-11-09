// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kernal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleNEO;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleSim;

public class RobotContainer {

  SwerveModuleSim sms;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final SwerveDrive swDrive;
  private final SwervePoseEstimator swPoseEstimator;

  public RobotContainer() {

    // Setup controllers depending on the current mode
    switch (Constants.kCurrentMode) {
      case REAL:
        if (!RobotBase.isReal()) {
          DriverStation.reportError("Attempted to run REAL on SIMULATED robot!", false);
          throw new NoSuchMethodError("Attempted to run REAL on SIMULATED robot!");
        }
        swDrive =
            new SwerveDrive(
                new SwerveModuleNEO(
                    SwerveConstants.kLeftFrontDriveMotorID,
                    SwerveConstants.kLeftFrontTurnMotorID,
                    SwerveConstants.kLeftFrontAnalogEncoderPort,
                    SwerveConstants.kLeftFrontModuleOffset,
                    SwerveConstants.kLeftFrontTurnMotorInverted,
                    SwerveConstants.kLeftFrontDriveMotorInverted),
                new SwerveModuleNEO(
                    SwerveConstants.kRightFrontDriveMotorID,
                    SwerveConstants.kRightFrontTurnMotorID,
                    SwerveConstants.kRightFrontAnalogEncoderPort,
                    SwerveConstants.kRightFrontModuleOffset,
                    SwerveConstants.kRightFrontTurnMotorInverted,
                    SwerveConstants.kRightFrontDriveMotorInverted),
                new SwerveModuleNEO(
                    SwerveConstants.kLeftRearDriveMotorID,
                    SwerveConstants.kLeftRearTurnMotorID,
                    SwerveConstants.kLeftRearAnalogEncoderPort,
                    SwerveConstants.kLeftRearModuleOffset,
                    SwerveConstants.kLeftRearTurnMotorInverted,
                    SwerveConstants.kLeftRearDriveMotorInverted),
                new SwerveModuleNEO(
                    SwerveConstants.kRightRearDriveMotorID,
                    SwerveConstants.kRightRearTurnMotorID,
                    SwerveConstants.kRightRearAnalogEncoderPort,
                    SwerveConstants.kRightRearModuleOffset,
                    SwerveConstants.kRightRearTurnMotorInverted,
                    SwerveConstants.kRightRearDriveMotorInverted));
        break;

      case SIMULATOR:
        if (RobotBase.isReal()) {
          DriverStation.reportError("Attempted to run SIMULATED on REAL robot!", false);
          throw new NoSuchMethodError("Attempted to run SIMULATED on REAL robot!");
        }
        swDrive =
            new SwerveDrive(
                new SwerveModuleSim(false),
                new SwerveModuleSim(false),
                new SwerveModuleSim(false),
                new SwerveModuleSim(false));

        break;

      default:
        throw new NoSuchMethodError("Not Implemented");
    }

    swPoseEstimator = new SwervePoseEstimator(swDrive);

    configureBindings();
  }

  private void configureBindings() {
    System.out.print("example");
    swDrive.setDefaultCommand(
        new SwerveJoystickCmd(
            swDrive,
            swPoseEstimator,
            () -> (-MathUtil.applyDeadband(controller.getLeftY(), ControllerConstants.kDeadband)),
            () -> (-MathUtil.applyDeadband(controller.getLeftX(), ControllerConstants.kDeadband)),
            () -> (-MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.kDeadband)),
            () -> (true)));

    // swDrive.setDefaultCommand(new SwerveDriveToTag(swDrive, swPoseEstimator));

    // new JoystickButton(controller.getHID(), Button.kX.value)
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             new InstantCommand(
    //                 () -> {
    //                   swDrive.resetGyro();
    //                   swPoseEstimator.reset(new Pose2d(0, 0, new Rotation2d()));
    //                 })));

    // new JoystickButton(controller.getHID(), Button.kY.value)
    //     .onTrue(new SwerveFollowSquare(swDrive, swPoseEstimator));

    // swDrive.setDefaultCommand(new SwerveSolveFeedForward(swDrive));

    // PathPlannerPath path = PathPlannerPath.fromPathFile("around the station");
    // HashMap<String, Command> eventMap = new HashMap<>();
    // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("around the station", new
    // PathConstraints(4, 3));
    // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swPoseEstimator::getPose,
    // swPoseEstimator::reset, new PIDConstants(5,0,0), new PIDConstants(0.5,0,0),
    // swDrive::setChassisSpeeds, eventMap, swDrive);

    // new JoystickButton(controller.getHID(), Button.kY.value)
    //     .onTrue(
    //         autoBuilder.fullAuto(pathGroup));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
