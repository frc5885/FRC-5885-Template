package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.WCLogger;
import frc.robot.base.RobotSystem;
import frc.robot.base.io.Beambreak;
import frc.robot.base.subsystems.SubsystemAction;
import frc.robot.base.subsystems.WCStaticSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

import edu.wpi.first.wpilibj.Encoder;

public class WristFFSubsystem extends WCStaticSubsystem {

  private final double buffer = 0.0;

  private SparkAbsoluteEncoder m_absoluteEncoder;
  private CANSparkMax m_wrist;
  private double m_setPoint = Constants.kWristStow;

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  @Override
  protected List<MotorController> initMotors() {
    m_wrist = new CANSparkMax(Constants.kWrist, MotorType.kBrushless);
    m_absoluteEncoder = m_wrist.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    return List.of(m_wrist);
  }

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  // Create a new SysId routine for characterizing the shooter.
  Measure<Velocity<Voltage>> rampRate = Volts.of(0.2).per(Seconds.of(1));
  Measure<Voltage> stepVoltage = Volts.of(3.0);
  Measure<Time> timeout = Seconds.of(10);
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.

          new SysIdRoutine.Config(rampRate, stepVoltage, timeout),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                m_wrist.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("feedforward-wrist")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_wrist.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(m_absoluteEncoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(m_absoluteEncoder.getVelocity(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));
  // PID controller to run the shooter wheel in closed-loop, set the constants equal to those
  // calculated by SysId
  // private final PIDController m_shooterFeedback = new PIDController(ShooterConstants.kP, 0, 0);
  // Feedforward controller to run the shooter wheel in closed-loop, set the constants equal to
  // those calculated by SysId
  // private final SimpleMotorFeedforward m_shooterFeedforward =
  //     new SimpleMotorFeedforward(
  //         ShooterConstants.kSVolts,
  //         ShooterConstants.kVVoltSecondsPerRotation,
  //         ShooterConstants.kAVoltSecondsSquaredPerRotation);

  /** Creates a new Shooter subsystem. */
  // public Shooter() {
  //   // Sets the distance per pulse for the encoders
  //   m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
  // }

  /**
   * Returns a command that runs the shooter at a specifc velocity.
   *
   * @param shooterSpeed The commanded shooter wheel speed in rotations per second
   */
  // public Command runShooter(DoubleSupplier shooterSpeed) {
  //   // Run shooter wheel at the desired speed using a PID controller and feedforward.
  //   return run(() -> {
  //         m_shooterMotor.setVoltage(
  //             m_shooterFeedback.calculate(m_shooterEncoder.getRate(), shooterSpeed.getAsDouble())
  //                 + m_shooterFeedforward.calculate(shooterSpeed.getAsDouble()));
  //         m_feederMotor.set(ShooterConstants.kFeederSpeed);
  //       })
  //       .finallyDo(
  //           () -> {
  //             m_shooterMotor.stopMotor();
  //             m_feederMotor.stopMotor();
  //           })
  //       .withName("runShooter");
  // }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return new SequentialCommandGroup(new InstantCommand(() -> System.out.println("Quasi")), m_sysIdRoutine.quasistatic(direction));
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
