// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kernal;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleIO;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleNEO;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveModuleSim;

public class RobotContainer {

  private final SwerveDrive m_swerve;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {

    // Setup controllers depending on the current mode
    switch (Constants.currentMode) {
      case REAL:
        if (!RobotBase.isReal())
          DriverStation.reportError("Attempted to run REAL on SIMULATED robot!", false);
        m_swerve =
            new SwerveDrive(
                new SwerveModuleNEO(1, 2, false, false, 0, false, 0),
                new SwerveModuleNEO(1, 2, false, false, 0, false, 0),
                new SwerveModuleNEO(1, 2, false, false, 0, false, 0),
                new SwerveModuleNEO(1, 2, false, false, 0, false, 0));
        break;

      case SIMULATOR:
        if (RobotBase.isReal())
          DriverStation.reportError("Attempted to run SIMULATED on REAL robot!", false);

        m_swerve =
            new SwerveDrive(
                new SwerveModuleSim(false, false, 0, false, 0),
                new SwerveModuleSim(false, false, 0, false, 0),
                new SwerveModuleSim(false, false, 0, false, 0),
                new SwerveModuleSim(false, false, 0, false, 0));
        break;

      default:
        m_swerve =
            new SwerveDrive(
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {});
        // m_swerve = new SwerveDrive(new SwerveModuleIO(), new SwerveModuleIO(), new
        // SwerveModuleIO(), new SwerveModuleIO());
        break;
    }

    configureBindings();
  }

  private void configureBindings() {
    m_swerve.setDefaultCommand(
        new SwerveJoystickCmd(
            m_swerve,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.getRightX(),
            () -> false));

    controller.a().onTrue(new InstantCommand(() -> m_swerve.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
