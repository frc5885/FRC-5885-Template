// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TankConstants;
import frc.robot.subsystems.TankDriveSubsystem.TankDrive;

public class TurnToAngle extends PIDCommand {
  /** Creates a new TurnToAngle. */
  public TurnToAngle(double angle, TankDrive tank) {
    super(
        // The controller that the command will use
        new PIDController(
            TankConstants.Auto.kTurnP, TankConstants.Auto.kTurnI, TankConstants.Auto.kTurnD),
        // This should return the measurement
        null,
        // This should return the setpoint (can also be a constant)
        angle,
        // This uses the output
        output -> {
          tank.driveTank(-output, output, TankConstants.Auto.kTurnMaxVoltage);
        },
        tank);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
