// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PoseEstimatorSubsystem;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;

/** Add your docs here. */
public class NoodleVision implements NoodleVisionIO {
  private final DoubleArraySubscriber m_observationSubscriber;
  private final DoubleSubscriber m_fpsSubscriber;

  /** Creates a new TankDrivePoseEstimator. */
  public NoodleVision(int id) {
    NetworkTable table =
        NetworkTableInstance.getDefault().getTable("NoodleVision" + id + "/output");
    m_observationSubscriber = table.getDoubleArrayTopic("observations").subscribe(new double[] {});
    m_fpsSubscriber = table.getDoubleTopic("fps").subscribe(0.0);
  }

  public void updateInputs(NoodleVisionIOInputs inputs) {
    TimestampedDoubleArray observation = m_observationSubscriber.getAtomic();
    inputs.observations = observation.value;
    inputs.timestamp = observation.timestamp;
    inputs.fps = m_fpsSubscriber.get();
  }
}
