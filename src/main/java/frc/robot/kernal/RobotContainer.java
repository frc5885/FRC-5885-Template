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
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleNEO;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleSim;

public class RobotContainer {

  SwerveModuleSim sms;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final SwerveDrive swDrive;

  public RobotContainer() {

    // Setup controllers depending on the current mode
    switch (Constants.currentMode) {
      case REAL:
        if (!RobotBase.isReal()) {
          DriverStation.reportError("Attempted to run REAL on SIMULATED robot!", false);
          throw new NoSuchMethodError("Attempted to run SIMULATED on REAL robot!");
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

    configureBindings();
  }

  private void configureBindings() {

    swDrive.setDefaultCommand(
        new SwerveJoystickCmd(
            swDrive,
            () -> (-MathUtil.applyDeadband(controller.getLeftY(), SwerveConstants.kDeadband)),
            () -> (-MathUtil.applyDeadband(controller.getLeftX(), SwerveConstants.kDeadband)),
            () -> (-MathUtil.applyDeadband(controller.getRightX(), SwerveConstants.kDeadband))));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
