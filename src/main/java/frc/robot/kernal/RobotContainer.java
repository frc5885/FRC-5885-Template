// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kernal;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.PerformSPath;
import frc.robot.subsystems.PoseEstimatorSubsystem.TankDrivePoseEstimator;
import frc.robot.subsystems.TankDriveSubsystem.TankDrive;
import frc.robot.subsystems.TankDriveSubsystem.TankDriveIO;
import frc.robot.subsystems.TankDriveSubsystem.TankDriveSim;
import frc.robot.subsystems.TankDriveSubsystem.TankDriveTalonSRX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final TankDrive m_tank;
  private final TankDrivePoseEstimator m_tankDrivePoseEstimator;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  public RobotContainer() {

    // Setup controllers depending on the current mode
    switch (Constants.currentMode) {
      case REAL:
        if (!RobotBase.isReal())
          DriverStation.reportError("Attempted to run REAL on SIMULATED robot!", false);
        m_tank = new TankDrive(new TankDriveTalonSRX());
        break;

      case SIMULATOR:
        if (RobotBase.isReal())
          DriverStation.reportError("Attempted to run SIMULATED on REAL robot!", false);

        m_tank = new TankDrive(new TankDriveSim());
        // throw new NoSuchMethodError("Not Implemented");
        break;

      default:
        m_tank = new TankDrive(new TankDriveIO() {});
        break;
    }

    m_tankDrivePoseEstimator =
        new TankDrivePoseEstimator(
            m_tank::getRotation, m_tank::getLeftPositionMeters, m_tank::getRightPositionMeters);

    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Perform path", new PerformSPath(m_tank, m_tankDrivePoseEstimator));
    autoChooser.addOption("Drive 10m", new DriveDistance(10.0, m_tank));
    // autoChooser.addOption("Turn 90 Degrees", new TurnToAngle(90.0, m_tank));

    configureBindings();
  }

  private void configureBindings() {
    m_tank.setDefaultCommand(
        new RunCommand(
            () -> {
              m_tank.driveTank(-controller.getLeftY(), -controller.getRightY(), 7);
            },
            m_tank));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
