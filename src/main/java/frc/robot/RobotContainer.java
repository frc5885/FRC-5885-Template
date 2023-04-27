// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.PerformSPath;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.tank.Tank;
import frc.robot.subsystems.tank.TankIO;
import frc.robot.subsystems.tank.TankSim;
import frc.robot.subsystems.tank.TankTalonSRX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final Tank m_tank;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        m_tank = new Tank(new TankTalonSRX());
        break;

      case SIM:
        m_tank = new Tank(new TankSim());
        // throw new NoSuchMethodError("Not Implemented");
        break;

      default:
        m_tank = new Tank(new TankIO() {});
        break;
    }

    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Perform path", new PerformSPath(m_tank));
    autoChooser.addOption("Drive 10m", new DriveDistance(10.0, m_tank));
    autoChooser.addOption("Turn 90 Degrees", new TurnToAngle(90.0, m_tank));

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
