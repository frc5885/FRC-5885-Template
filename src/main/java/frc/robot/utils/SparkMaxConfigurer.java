package frc.robot.utils;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

public class SparkMaxConfigurer {
  /**
   * Sets the default frame timings for a CANSparkMax motor.
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   *
   * @param motor the CANSparkMax motor to set the frame timings for
   */
  public static void setFrameTimingsDefault(CANSparkMax motor) {
    // Applied output, Faults, sticky faults, is follower
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    // Velocity, temperature, voltage, current
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    // Position
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    // Analog voltage, analog velocity, analog position
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
    // Alternate encoder velocity, alternate encoder position
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20);
    // Duty cycle absolute encoder position, duty cycle velocity
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);
    // Duty cycle Absolute encoder velocity, duty cycle alternate encoder frequency
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);
    // IAccum?? Not sure what this is, was added in 24.0.0 release, leave as default for now
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);
  }

  /**
   * Sets the frame timings for optimized performance.
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   *
   * @param motor the CANSparkMax motor to configure
   */
  public static void setFrameTimingsOptmized(CANSparkMax motor) {
    // Assuming we aren't using a follower, frame 0 can be set higher
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    // "Disable" extra encoder stuff
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);
  }

  /**
   * Sets the frame timings for an optimized follower CANSparkMax motor.
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   *
   * @param motor the CANSparkMax motor to configure
   */
  public static void setFrameTimingsOptmizedFollower(CANSparkMax motor) {
    // Assuming we are a follower, set frame 0 to be lower
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    // "Disable" extra encoder stuff
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 250);
  }
}
