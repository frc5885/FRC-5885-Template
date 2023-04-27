// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.TankConstants;
import frc.robot.subsystems.tank.Tank;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistance extends ProfiledPIDCommand {
  Tank m_tank;

  /** Creates a new DriveDistance. */
  public DriveDistance(double distanceMeters, Tank tank) {

    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            TankConstants.Auto.kPDriveVel,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                TankConstants.Auto.kDriveMaxVelocity, TankConstants.Auto.kDriveMaxAcceleration)),
        // This should return the measurement
        () -> {
          return tank.getAveragePositionMeters() - tank.getStoredDistanceMeters();
        },
        // This should return the goal (can also be a constant)
        distanceMeters,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          tank.setVoltage(output, output);
        },
        tank);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    getController().setTolerance(0.01);
    m_tank = tank;
  }

  @Override
  public void initialize() {
    getController().reset(0);
    m_tank.storeCurrentAverageDistance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
