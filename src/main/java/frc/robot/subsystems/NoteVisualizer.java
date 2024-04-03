package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.WCLogger;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer extends SubsystemBase {
  private WristSubsystem m_wristSubsystem;
  private Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);
  private Translation3d redSpeaker = new Translation3d(16.317, 5.55, 2.1);
  private Transform3d m_launcherTransform;
  private final double kShotSpeed = 2.0;
  private ArrayList<Pose3d> m_poses;
  private ArrayList<Double> m_durations;
  private ArrayList<Timer> m_timers;
  private Supplier<Pose2d> robotPoseSupplier;

  public NoteVisualizer(Supplier<Pose2d> supplier, WristSubsystem wristSubsystem) {
    m_poses = new ArrayList<>();
    m_durations = new ArrayList<>();
    m_timers = new ArrayList<>();
    robotPoseSupplier = supplier;
    m_wristSubsystem = wristSubsystem;
    if (WCLogger.isEnabled) {
      Logger.recordOutput("Note", new Pose3d[] {});
    }
  }

  @Override
  public void periodic() {
    if (WCLogger.isEnabled) {
      Logger.recordOutput("Note", m_poses.toArray(new Pose3d[m_poses.size()]));
    }
  }

  public void shoot() {
    new Thread(
            () -> {
              m_launcherTransform =
                  new Transform3d(0.35, 0, 0.8, m_wristSubsystem.getWristSimRotation());
              m_poses.add(new Pose3d(robotPoseSupplier.get()).transformBy(m_launcherTransform));
              int indexPose = m_poses.size() - 1;
              Pose3d endPose =
                  new Pose3d(
                      DriverStation.getAlliance().get() == Alliance.Red
                              && DriverStation.getAlliance().isPresent()
                          ? redSpeaker
                          : blueSpeaker,
                      m_poses.get(indexPose).getRotation());
              m_durations.add(
                  m_poses.get(indexPose).getTranslation().getDistance(endPose.getTranslation())
                      / kShotSpeed);
              int indexDuration = m_durations.size() - 1;
              m_timers.add(new Timer());
              int indexTimer = m_timers.size() - 1;
              m_timers.get(indexTimer).start();
              while (!m_timers.get(indexTimer).hasElapsed(m_durations.get(indexDuration))) {
                m_poses.set(
                    indexPose,
                    m_poses
                        .get(indexPose)
                        .interpolate(
                            endPose,
                            m_timers.get(indexTimer).get() / m_durations.get(indexDuration)));
                try {
                  Thread.sleep(20);
                } catch (InterruptedException e) {
                  e.printStackTrace();
                }
              }
              m_poses.remove(indexPose);
              m_durations.remove(indexDuration);
              m_timers.remove(indexTimer);
            })
        .start();
  }
}
