package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PoseEstimatorSubsystem.SwervePoseEstimator;

public class DriveTwoMeters extends SequentialCommandGroup {
  public DriveTwoMeters(SwervePoseEstimator poseEstimator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerPath path = PathPlannerPath.fromPathFile("two_meter");

    addCommands(
        new InstantCommand(
            () -> {
              poseEstimator.reset(path.getPreviewStartingHolonomicPose());
            }),
        AutoBuilder.followPath(path));
  }
}
