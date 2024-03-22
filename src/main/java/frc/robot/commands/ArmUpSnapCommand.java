package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.base.io.Beambreak;
import frc.robot.base.subsystems.swerve.SwerveAction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmUpSnapCommand extends ParallelCommandGroup {
    public ArmUpSnapCommand(
      ArmSubsystem armSubsystem,
      WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      Beambreak beambreak,
      Robot robot
    ) {
        super(
          new ArmUpCommand(
            armSubsystem,
            wristSubsystem,
            shooterSubsystem,
            feederSubsystem,
            beambreak
          ),
          new SetSwerveActionCommand(
            robot,
            SwerveAction.FACEAMP
          )
        );
    }
}