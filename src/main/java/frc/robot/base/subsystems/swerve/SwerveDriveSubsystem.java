// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.base.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.base.modules.swerve.*;

public class SwerveDriveSubsystem extends SubsystemBase {

  private final SwerveModule[] m_modules = new SwerveModule[4];
  private final AHRS m_gyro;
  // IO Modules can't be defined in constructor, so they are defined here
  private final frc.robot.base.modules.swerve.SwerveModule.SwerveModuleInput[] m_modulesInput = {
    new frc.robot.base.modules.swerve.SwerveModule.SwerveModuleInput(),
    new frc.robot.base.modules.swerve.SwerveModule.SwerveModuleInput(),
    new frc.robot.base.modules.swerve.SwerveModule.SwerveModuleInput(),
    new frc.robot.base.modules.swerve.SwerveModule.SwerveModuleInput()
  };

  private final PIDController[] m_turnController = new PIDController[4];
  private final PIDController[] m_driveController = new PIDController[4];
  private final SimpleMotorFeedforward[] m_driveFeedforward = new SimpleMotorFeedforward[4];

  // private Pose2d lastPos = new Pose2d();
  private double fieldXVel = 0;
  private double fieldYVel = 0;

  private Rotation2d m_heading = new Rotation2d(0);

  private SysIdRoutine m_sysIdRoutine;

  /** Creates a new SwerveDrive. */
  public SwerveDriveSubsystem() {
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;

    if (RobotBase.isReal()) {
      frontLeft =
          new NEOSwerveModule(
              SwerveConstants.Module.kLeftFrontDriveMotorID,
              SwerveConstants.Module.kLeftFrontTurnMotorID,
              SwerveConstants.Module.kLeftFrontAnalogEncoderPort,
              SwerveConstants.Module.kLeftFrontModuleOffset,
              SwerveConstants.Module.kLeftFrontTurnMotorInverted,
              SwerveConstants.Module.kLeftFrontDriveMotorInverted);
      frontRight =
          new NEOSwerveModule(
              SwerveConstants.Module.kRightFrontDriveMotorID,
              SwerveConstants.Module.kRightFrontTurnMotorID,
              SwerveConstants.Module.kRightFrontAnalogEncoderPort,
              SwerveConstants.Module.kRightFrontModuleOffset,
              SwerveConstants.Module.kRightFrontTurnMotorInverted,
              SwerveConstants.Module.kRightFrontDriveMotorInverted);
      backLeft =
          new NEOSwerveModule(
              SwerveConstants.Module.kLeftRearDriveMotorID,
              SwerveConstants.Module.kLeftRearTurnMotorID,
              SwerveConstants.Module.kLeftRearAnalogEncoderPort,
              SwerveConstants.Module.kLeftRearModuleOffset,
              SwerveConstants.Module.kLeftRearTurnMotorInverted,
              SwerveConstants.Module.kLeftRearDriveMotorInverted);
      backRight =
          new NEOSwerveModule(
              SwerveConstants.Module.kRightRearDriveMotorID,
              SwerveConstants.Module.kRightRearTurnMotorID,
              SwerveConstants.Module.kRightRearAnalogEncoderPort,
              SwerveConstants.Module.kRightRearModuleOffset,
              SwerveConstants.Module.kRightRearTurnMotorInverted,
              SwerveConstants.Module.kRightRearDriveMotorInverted);
    } else {
      frontLeft = new SimulatedSwerveModule(false);
      frontRight = new SimulatedSwerveModule(false);
      backLeft = new SimulatedSwerveModule(false);
      backRight = new SimulatedSwerveModule(false);
    }

    m_gyro = new AHRS(SPI.Port.kMXP);
    resetGyro();

    m_modules[0] = frontLeft;
    m_modules[1] = frontRight;
    m_modules[2] = backLeft;
    m_modules[3] = backRight;

    for (int i = 0; i != 4; i++) {
      m_turnController[i] =
          new PIDController(
              SwerveConstants.Module.kTurningFeedbackP,
              SwerveConstants.Module.kTurningFeedbackI,
              SwerveConstants.Module.kTurningFeedbackD);
      m_turnController[i].enableContinuousInput(-Math.PI, Math.PI);
      m_turnController[i].setTolerance(SwerveConstants.Module.kTurningFeedbackTolerance);

      if (RobotBase.isReal()) {
        m_driveController[i] =
            new PIDController(
                SwerveConstants.Module.kDriveFeedbackP,
                SwerveConstants.Module.kDriveFeedbackI,
                SwerveConstants.Module.kDriveFeedbackD);
        m_driveFeedforward[i] =
            new SimpleMotorFeedforward(
                SwerveConstants.Module.kDriveFeedForwardKs,
                SwerveConstants.Module.kDriveFeedForwardKv,
                SwerveConstants.Module.kDriveFeedForwardKa);
      } else {
        m_driveController[i] =
            new PIDController(
                SwerveConstants.Module.Simulation.kDriveFeedbackP,
                SwerveConstants.Module.Simulation.kDriveFeedbackI,
                SwerveConstants.Module.Simulation.kDriveFeedbackD);
        m_driveFeedforward[i] =
            new SimpleMotorFeedforward(
                SwerveConstants.Module.Simulation.kDriveFeedForwardKs,
                SwerveConstants.Module.Simulation.kDriveFeedForwardKv,
                SwerveConstants.Module.Simulation.kDriveFeedForwardKa);
      }
    }

    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::sysidSetVoltageDrive, this::sysidGetLog, this));
  }

  @Override
  public void periodic() {
    for (int i = 0; i != 4; i++) {
      m_modules[i].updateInputs(m_modulesInput[i]);
      // Logger.eprocessInputs("SwerveDrive/Modules/Module" + Integer.toString(i),
      // m_modulesInput[i]);
    }

    var chassisSpeeds = SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    double chassisRotationSpeed = chassisSpeeds.omegaRadiansPerSecond;

    if (RobotBase.isReal()) {
      m_heading = Rotation2d.fromDegrees(-m_gyro.getAngle());
    } else {
      m_heading = m_heading.plus(Rotation2d.fromRadians(chassisRotationSpeed * 0.02));
    }

    // Logger.recordOutput("SwerveDrive/currentModuleStates", getModuleStates());
    // Logger.recordOutput("SwerveDrive/headingDegrees", m_heading.getDegrees());
    // Logger.recordOutput("SwerveDrive/headingRadians", m_heading.getRadians());
  }

  public Pose2d getFieldVelocity() {
    return new Pose2d(fieldXVel, fieldYVel, getRotation2d());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(
          m_modulesInput[0].drivePositionMeters, new Rotation2d(m_modulesInput[0].turnPositionRad)),
      new SwerveModulePosition(
          m_modulesInput[1].drivePositionMeters, new Rotation2d(m_modulesInput[1].turnPositionRad)),
      new SwerveModulePosition(
          m_modulesInput[2].drivePositionMeters, new Rotation2d(m_modulesInput[2].turnPositionRad)),
      new SwerveModulePosition(
          m_modulesInput[3].drivePositionMeters, new Rotation2d(m_modulesInput[3].turnPositionRad)),
    };
  }

  public double getHeading() {
    // return m_rotation;
    return Math.IEEEremainder(m_heading.getDegrees(), 360);
  }

  public Rotation2d getRotation2d() {
    return m_heading;
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      new SwerveModuleState(
          m_modulesInput[0].driveVelocityMetersPerSec,
          new Rotation2d(m_modulesInput[0].turnPositionRad)),
      new SwerveModuleState(
          m_modulesInput[1].driveVelocityMetersPerSec,
          new Rotation2d(m_modulesInput[1].turnPositionRad)),
      new SwerveModuleState(
          m_modulesInput[2].driveVelocityMetersPerSec,
          new Rotation2d(m_modulesInput[2].turnPositionRad)),
      new SwerveModuleState(
          m_modulesInput[3].driveVelocityMetersPerSec,
          new Rotation2d(m_modulesInput[3].turnPositionRad)),
    };
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.kAttainableMaxSpeedMetersPerSecond);

    for (int i = 0; i != 4; i++) {
      if (Math.abs(desiredStates[i].speedMetersPerSecond) < 0.001) {
        m_modules[i].setDriveVoltage(0);
      }

      // In case of a sharp wheel turn, this helps prevent the
      // innertia of the robot from sliding too much.
      desiredStates[i].speedMetersPerSecond *= Math.cos(m_turnController[i].getPositionError());

      desiredStates[i] =
          SwerveModuleState.optimize(
              desiredStates[i], new Rotation2d(m_modulesInput[i].turnPositionRad));

      m_modules[i].setDriveVoltage(
          m_driveFeedforward[i].calculate(desiredStates[i].speedMetersPerSecond)
              + m_driveController[i].calculate(
                  m_modulesInput[i].driveVelocityMetersPerSec,
                  desiredStates[i].speedMetersPerSecond));

      m_modules[i].setTurnVoltage(
          (m_turnController[i].calculate(
              m_modulesInput[i].turnPositionRad, desiredStates[i].angle.getRadians())));
    }
  }

  public void resetGyro() {
    m_gyro.reset();
    m_heading = Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public void setModulesAngle(double angle) {

    for (int i = 0; i != 4; i++) {
      m_modules[i].setTurnVoltage(
          (m_turnController[i].calculate(m_modulesInput[i].turnPositionRad, angle)));
    }
  }

  public void setVoltage(double voltage) {
    for (int i = 0; i != 4; i++) {
      m_modules[i].setDriveVoltage(voltage);
    }
  }

  private void sysidSetVoltageDrive(Measure<Voltage> volts) {
    for (int i = 0; i != 4; i++) {
      setModulesAngle(0.0);
      m_modules[i].setDriveVoltage(volts.in(Units.Volts));
    }
  }

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage =
      MutableMeasure.mutable(Units.Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = MutableMeasure.mutable(Units.Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity =
      MutableMeasure.mutable(Units.MetersPerSecond.of(0));

  private SysIdRoutineLog sysidGetLog(SysIdRoutineLog log) {
    log.motor("drive-left")
        .voltage(
            m_appliedVoltage.mut_replace(
                (m_modulesInput[0].driveVoltage + m_modulesInput[3].driveVoltage) / 2.0,
                Units.Volts))
        .linearPosition(
            m_distance.mut_replace(
                (m_modulesInput[0].drivePositionMeters + m_modulesInput[3].drivePositionMeters)
                    / 2.0,
                Units.Meters))
        .linearVelocity(
            m_velocity.mut_replace(
                (m_modulesInput[0].driveVelocityMetersPerSec
                        + m_modulesInput[3].driveVelocityMetersPerSec)
                    / 2.0,
                Units.MetersPerSecond));

    log.motor("drive-right")
        .voltage(
            m_appliedVoltage.mut_replace(
                (m_modulesInput[1].driveVoltage + m_modulesInput[2].driveVoltage) / 2.0,
                Units.Volts))
        .linearPosition(
            m_distance.mut_replace(
                (m_modulesInput[1].drivePositionMeters + m_modulesInput[2].drivePositionMeters)
                    / 2.0,
                Units.Meters))
        .linearVelocity(
            m_velocity.mut_replace(
                (m_modulesInput[1].driveVelocityMetersPerSec
                        + m_modulesInput[2].driveVelocityMetersPerSec)
                    / 2.0,
                Units.MetersPerSecond));

    return log;
  }

  public Command getSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command getSysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

  public double getAverageMotorVoltage() {
    return (m_modulesInput[0].driveVoltage
            + m_modulesInput[1].driveVoltage
            + m_modulesInput[2].driveVoltage
            + m_modulesInput[3].driveVoltage)
        / 4;
  }

  public double getAngularVelocity() {
    return getChassisSpeeds().omegaRadiansPerSecond;
  }

  public double getAbsoluteEncoderValue(int index) {
    if (index < 0 || index > 3) {
      return 0.0;
    }
    return m_modulesInput[index].turnAbsolutePositionRad;
  }

  public void stop() {
    for (int i = 0; i != 4; i++) {
      m_modules[i].setDriveVoltage(0);
      m_modules[i].setTurnVoltage(0);
    }
  }
}
