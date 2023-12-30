package frc.robot.utils;

import com.revrobotics.CANSparkMax;

public class SparkMaxConfigurer {
  /**
   * Sets the default frame timings for a CANSparkMax motor.
   *
   * @param motor the CANSparkMax motor to set the frame timings for
   */
  public static void setFrameTimingsDefault(CANSparkMax motor) {
    // Applied output, Faults, sticky faults, is follower
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 10);
    // Velocity, temperature, voltage, current
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
    // Position
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);
    // Analog voltage, analog velocity, analog position
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus3, 50);
    // Alternate encoder velocity, alternate encoder position
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus4, 20);
    // Duty cycle absolute encoder position, duty cycle velocity
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus5, 200);
    // Duty cycle Absolute encoder velocity, duty cycle alternate encoder frequency
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus6, 200);
  }

  /**
   * Sets the frame timings for optimized performance.
   *
   * @param motor the CANSparkMax motor to configure
   */
  public static void setFrameTimingsOptmized(CANSparkMax motor) {
    // Assuming we aren't using a follower, frame 0 can be set higher
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);
    // "Disable" extra encoder stuff
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus3, 65535);
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus4, 65535);
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus5, 65535);
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus6, 65535);
  }

  /**
   * Sets the frame timings for an optimized follower CANSparkMax motor.
   *
   * @param motor the CANSparkMax motor to configure
   */
  public static void setFrameTimingsOptmizedFollower(CANSparkMax motor) {
    // Assuming we are a follower, set frame 0 to be lower
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 10);
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);
    // "Disable" extra encoder stuff
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus3, 65535);
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus4, 65535);
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus5, 65535);
    motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus6, 65535);
  }
}
