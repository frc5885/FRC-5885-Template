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
import frc.robot.subsystems.PoseEstimatorSubsystem.TankDrivePoseEstimator;
import frc.robot.subsystems.TankDriveSubsystem.TankDrive;
import frc.robot.subsystems.TankDriveSubsystem.TankDriveIO;
import frc.robot.subsystems.TankDriveSubsystem.TankDriveSim;

public class RobotContainer {

  private final TankDrive m_tank;
  private final TankDrivePoseEstimator m_poseEstimator;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {

    // Setup controllers depending on the current mode
    switch (Constants.currentMode) {
      case REAL:
        if (!RobotBase.isReal())
          DriverStation.reportError("Attempted to run REAL on SIMULATED robot!", false);
        m_tank = new TankDrive(new TankDriveIO() {});
        break;

      case SIMULATOR:
        if (RobotBase.isReal())
          DriverStation.reportError("Attempted to run SIMULATED on REAL robot!", false);

        m_tank = new TankDrive(new TankDriveSim());
        break;

      default:
        m_tank = new TankDrive(new TankDriveIO() {});
        // m_swerve = new SwerveDrive(new SwerveModuleIO(), new SwerveModuleIO(), new
        // SwerveModuleIO(), new SwerveModuleIO());
        break;
    }

    m_poseEstimator =
        new TankDrivePoseEstimator(
            m_tank::getRotation, m_tank::getLeftPositionMeters, m_tank::getRightPositionMeters);

    configureBindings();
  }

  private void configureBindings() {
    m_tank.setDefaultCommand(
        new InstantCommand(
            () -> {
              m_tank.driveTank(-controller.getLeftY(), -controller.getRightY(), 7);
            },
            m_tank));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
