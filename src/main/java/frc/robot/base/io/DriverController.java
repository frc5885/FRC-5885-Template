package frc.robot.base.io;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DriverController extends WCXboxController {

    public DriverController(
        Command xCommand,
        Command yCommand
    ) {
        super(ControllerConstants.kDriverControllerPort);
        xButton.onTrue(xCommand);
        yButton.onTrue(yCommand);
    }

    @Override
    public JoystickButton getXButton() {
        throw new RuntimeException("X is reserved for resetting the swerve gyro!");
    }

    @Override
    public JoystickButton getYButton() {
        throw new RuntimeException("Y is reserved for changing swerve field orientation!");
    }
}