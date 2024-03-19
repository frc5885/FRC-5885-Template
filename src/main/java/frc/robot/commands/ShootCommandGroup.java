package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ShootCommandGroup extends SequentialCommandGroup{
    
    public ShootCommandGroup(FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem){
        addCommands(
            new ShootCommand(feederSubsystem, shooterSubsystem),
            new ArmDownCommand(armSubsystem, wristSubsystem)
        );
    }

}
