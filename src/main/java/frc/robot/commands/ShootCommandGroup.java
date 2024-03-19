package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.base.io.Beambreak;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ShootCommandGroup extends SequentialCommandGroup{
    
    // TODO turn off aimbot after shot

    public ShootCommandGroup(FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, Beambreak beambreak){
        addCommands(
            new ShootCommand(feederSubsystem, shooterSubsystem, beambreak),
            new ArmDownCommand(armSubsystem, wristSubsystem)
        );
    }

}
