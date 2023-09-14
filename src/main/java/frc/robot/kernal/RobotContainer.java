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
        if (!RobotBase.isReal())
          DriverStation.reportError("Attempted to run REAL on SIMULATED robot!", false);

        break;

      case SIMULATOR:
        if (RobotBase.isReal())
          DriverStation.reportError("Attempted to run SIMULATED on REAL robot!", false);

        // throw new NoSuchMethodError("Not Implemented");
        break;

      default:
        break;
    }

    swDrive = new SwerveDrive();

    configureBindings();
  }

  private void configureBindings() {

    swDrive.setDefaultCommand(
        new SwerveJoystickCmd(
            swDrive,
            () -> (-controller.getLeftY()*0.5),
            () -> (-controller.getLeftX()*0.5),
            () -> (-controller.getRightX())*0.75));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
