package frc.robot.subsystems.PoseEstimatorSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface NoodleVisionIO {

  @AutoLog
  public static class NoodleVisionIOInputs {
    public double[] observations = new double[] {};
    public double timestamp = 0.0;
    public double fps = 0;
  }

  public default void updateInputs(NoodleVisionIOInputs inputs) {}
}
