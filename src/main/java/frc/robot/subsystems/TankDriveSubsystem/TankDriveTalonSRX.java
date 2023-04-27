package frc.robot.subsystems.TankDriveSubsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.TankConstants;

public class TankDriveTalonSRX implements TankDriveIO {

  private final WPI_TalonSRX m_leftFrontMotor;
  private final WPI_TalonSRX m_leftRearMotor;
  private final WPI_TalonSRX m_rightFrontMotor;
  private final WPI_TalonSRX m_rightRearMotor;

  private final MotorControllerGroup m_leftMotors;
  private final MotorControllerGroup m_rightMotors;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final AHRS m_gyro;

  public TankDriveTalonSRX() {
    m_leftFrontMotor = new WPI_TalonSRX(TankConstants.kLeftFrontMotorID);
    m_leftRearMotor = new WPI_TalonSRX(TankConstants.kLeftRearMotorID);
    m_rightFrontMotor = new WPI_TalonSRX(TankConstants.kRightFrontMotorID);
    m_rightRearMotor = new WPI_TalonSRX(TankConstants.kRightRearMotorID);

    m_leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftRearMotor);
    m_rightMotors = new MotorControllerGroup(m_rightFrontMotor, m_rightRearMotor);

    m_leftMotors.setInverted(TankConstants.kLeftMotorsInverted);
    m_rightMotors.setInverted(TankConstants.kRightMotorsInverted);

    m_leftEncoder =
        new Encoder(
            TankConstants.kLeftEncoderAPort,
            TankConstants.kLeftEncoderBPort,
            TankConstants.kLeftEncoderInverted);
    m_rightEncoder =
        new Encoder(
            TankConstants.kRightEncoderAPort,
            TankConstants.kRightEncoderBPort,
            TankConstants.kRightEncoderInverted);

    m_leftEncoder.setDistancePerPulse(TankConstants.kEncoderMetersPerPulse);
    m_rightEncoder.setDistancePerPulse(TankConstants.kEncoderMetersPerPulse);

    m_gyro = new AHRS(SPI.Port.kMXP);
  }

  @Override
  public void updateInputs(TankDriveIOInputs inputs) {
    inputs.leftPositionMeters = m_leftEncoder.getDistance();
    inputs.leftVelocityMetersPerSec = m_leftEncoder.getRate();
    inputs.rightPositionMeters = m_rightEncoder.getDistance();
    inputs.rightVelocityMetersPerSec = m_rightEncoder.getRate();
    inputs.gyroRotation = m_gyro.getRotation2d();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
  }

  @Override
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
}
