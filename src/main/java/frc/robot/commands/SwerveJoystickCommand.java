// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.WCLogger;
import frc.robot.base.modules.swerve.SwerveConstants;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.base.subsystems.swerve.SwerveAction;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SwerveJoystickCommand extends Command {

  private final SwerveDriveSubsystem m_swerveSubsystem;
  private final SwervePoseEstimator m_poseEstimator;
  private final PhotonVisionSystem m_photonVision;
  private final Supplier<Double> m_xDrivePercentFunction,
      m_yDrivePercentFunction,
      m_turnDrivePercentFunction;
  private final Supplier<Boolean> m_fieldOrientedFunction;
  // private final Supplier<Boolean> m_aimBotFunction;
  private final Supplier<SwerveAction> m_swerveActionFuntion;

  private PIDController m_aimBotPID;
  private PIDController m_facingPID;

  //  private static final LoggedDashboardChooser<Double> m_linearSpeedLimitChooser =
  //      new LoggedDashboardChooser<>("Linear Speed Limit");
  //  private static final LoggedDashboardChooser<Double> m_angularSpeedLimitChooser =
  //      new LoggedDashboardChooser<>("Angular Speed Limit");
  //
  //  static {
  //    m_linearSpeedLimitChooser.addDefaultOption("100%", 1.0);
  //    m_linearSpeedLimitChooser.addOption("75%", 0.75);
  //    m_linearSpeedLimitChooser.addOption("50%", 0.5);
  //    m_linearSpeedLimitChooser.addOption("25%", 0.25);
  //    m_angularSpeedLimitChooser.addDefaultOption("100%", 1.0);
  //    m_angularSpeedLimitChooser.addOption("75%", 0.75);
  //    m_angularSpeedLimitChooser.addOption("50%", 0.5);
  //    m_angularSpeedLimitChooser.addOption("25%", 0.25);
  //  }

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCommand(
      SwerveDriveSubsystem swerveSubsystem,
      SwervePoseEstimator poseEstimator,
      PhotonVisionSystem photonVision,
      Supplier<Double> xDrivePercentFunction,
      Supplier<Double> yDrivePercentFunction,
      Supplier<Double> turnDrivePercentFunction,
      Supplier<Boolean> fieldOrientedFunction,
      // Supplier<Boolean> aimBotFunction,
      Supplier<SwerveAction> swerveActionFunction) {
    m_swerveSubsystem = swerveSubsystem;
    m_poseEstimator = poseEstimator;
    m_photonVision = photonVision;
    m_xDrivePercentFunction = xDrivePercentFunction;
    m_yDrivePercentFunction = yDrivePercentFunction;
    m_turnDrivePercentFunction = turnDrivePercentFunction;
    m_fieldOrientedFunction = fieldOrientedFunction;
    // m_aimBotFunction = aimBotFunction;
    m_swerveActionFuntion = swerveActionFunction;

    // F = 1.54hz
    m_aimBotPID =
        new PIDController(
            SwerveConstants.AimBotConstants.kAimbotP,
            SwerveConstants.AimBotConstants.kAimbotI,
            SwerveConstants.AimBotConstants.kAimbotD);
    m_aimBotPID.enableContinuousInput(-Math.PI, Math.PI);
    m_aimBotPID.setTolerance(SwerveConstants.AimBotConstants.kAimbotTolerance);
    m_facingPID = new PIDController(2, 0, 0.2);
    //         SwerveConstants.AimBotConstants.kAimbotP,
    //         SwerveConstants.AimBotConstants.kAimbotI,
    //         SwerveConstants.AimBotConstants.kAimbotD);
    // m_facingPID.enableContinuousInput(-Math.PI, Math.PI);
    m_facingPID.setTolerance(SwerveConstants.AimBotConstants.kAimNoteTolerance);
    WCLogger.putData(this, "FacingPID", m_facingPID);
    WCLogger.putData(this, "AimbotPID", m_aimBotPID);

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xDir = m_xDrivePercentFunction.get();
    double yDir = m_yDrivePercentFunction.get();

    // Using the deadband here instead of the controllers
    // improves the control over the direction of the robot
    // since we no longer snap to the x/y axis.
    //
    // Another proble is that the magnitude of the x/y axis
    // is not always 1.0, so we need to cap it to fix that.
    // Ideally we would normalize the values but the max values
    // are not always the same on every controller and position.
    double magnitude = MathUtil.clamp(Math.hypot(xDir, yDir), -1.0, 1.0);
    magnitude = MathUtil.applyDeadband(magnitude, SwerveConstants.kSwerveDriveDeadband);

    // Square the magnitude (0-1), this gives us better control at slow speeds and
    // lets us go fast when we want to. This is a per person preferance and might
    // need to be changed.
    double magnitudeSqrd = Math.pow(magnitude, 2);

    double linearVelocity =
        magnitudeSqrd
            * SwerveConstants.kMaxSpeedMetersPerSecond
            * 1.0; // m_linearSpeedLimitChooser.get();
    Rotation2d linearDirection = new Rotation2d(xDir, yDir);

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    // Rotation speed stuff
    double angularVelocity;
    double rightJoystickX = m_turnDrivePercentFunction.get();
    Pose2d robotPose = m_poseEstimator.getPose();

    switch (m_swerveActionFuntion.get()) {
      case DEFAULT:
        angularVelocity =
            MathUtil.applyDeadband(rightJoystickX, SwerveConstants.kSwerveDriveDeadband);
        break;
      case AIMBOTTING:
        double angleToTarget =
            m_photonVision.getAngleToTarget(robotPose, m_photonVision.getTargetID());
        angularVelocity =
            m_aimBotPID.calculate(robotPose.getRotation().getRadians(), angleToTarget);
        if (m_aimBotPID.atSetpoint()) {
          angularVelocity = 0;
        }
        break;
      case AUTOAIMNOTE:
        angularVelocity = m_facingPID.calculate(m_photonVision.getAngleToNote(), 0);
        yDir = -1;
        // m_swerveSubsystem.setModuleStates(
        // SwerveConstants.kDriveKinematics.toSwerveModuleStates(
        //     ChassisSpeeds.fromRobotRelativeSpeeds(SwerveConstants.kMaxSpeedMetersPerSecond, 0, 0,
        // Rotation2d.fromDegrees(0))));
        if (m_facingPID.atSetpoint()) {
          angularVelocity = 0;
        }
        break;
      case AIMNOTE:
        angularVelocity = m_facingPID.calculate(m_photonVision.getAngleToNote(), 0);
        if (m_facingPID.atSetpoint()) {
          angularVelocity = 0;
        }
        break;
      case FACEFORWARD:
        angularVelocity =
            m_aimBotPID.calculate(
                robotPose.getRotation().getRadians(), alliance == Alliance.Blue ? 0 : Math.PI);
        if (m_aimBotPID.atSetpoint()) {
          angularVelocity = 0;
        }
        break;
      case FACEBACKWARD:
        angularVelocity =
            m_aimBotPID.calculate(
                robotPose.getRotation().getRadians(), alliance == Alliance.Blue ? Math.PI : 0);
        if (m_aimBotPID.atSetpoint()) {
          angularVelocity = 0;
        }
        break;
      case FACEAMP:
        angularVelocity =
            m_aimBotPID.calculate(
                robotPose.getRotation().getRadians(),
                alliance == Alliance.Blue ? Math.PI / 2 : Math.PI / 2);
        if (m_aimBotPID.atSetpoint()) {
          angularVelocity = 0;
        }
        break;
      case FACESOURCE:
        angularVelocity =
            m_aimBotPID.calculate(
                robotPose.getRotation().getRadians(),
                alliance == Alliance.Blue ? Math.PI * 2 / 3 : Math.PI / 3);
        if (m_aimBotPID.atSetpoint()) {
          angularVelocity = 0;
        }
        break;
      case PASS:
        angularVelocity =
            m_aimBotPID.calculate(
                robotPose.getRotation().getRadians(),
                m_photonVision.getAngleToPose(
                    m_poseEstimator.getPose(),
                    alliance == Alliance.Blue
                        ? Constants.kPassTargetBlue
                        : Constants.kPassTargetRed));
        if (m_aimBotPID.atSetpoint()) {
          angularVelocity = 0;
        }
        break;
      default:
        angularVelocity =
            MathUtil.applyDeadband(rightJoystickX, SwerveConstants.kSwerveDriveDeadband);
        break;
    }

    // if (m_aimBotFunction.get()) {
    // double angleToTarget = m_photonVision.getAngleToTarget(robotPose,
    // m_photonVision.getTargetID());
    // angularVelocity = m_aimBotPID.calculate(robotPose.getRotation().getRadians(),
    // angleToTarget);
    // if (m_aimBotPID.atSetpoint()) {
    // angularVelocity = 0;
    // }
    // } else {
    // angularVelocity = MathUtil.applyDeadband(rightJoystickX,
    // SwerveConstants.kSwerveDriveDeadband);
    // }

    angularVelocity *=
        SwerveConstants.kMaxSpeedAngularRadiansPerSecond * 1.0; // m_angularSpeedLimitChooser.get();

    // This does the trig for us and lets us get the x/y velocity
    Translation2d translation = new Translation2d(linearVelocity, linearDirection);
    ChassisSpeeds chassisSpeeds;

    // Use field oriented drive
    if (m_fieldOrientedFunction.get()) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(),
              translation.getY(),
              angularVelocity,
              m_poseEstimator // This is used to compensate for skew when driving and turning.
                  // No idea how this works, but it does.
                  .getPose()
                  .getRotation()
                  .plus(
                      new Rotation2d(
                          m_swerveSubsystem.getAngularVelocity() * SwerveConstants.kDriftFactor))
                  .plus(
                      new Rotation2d(
                          Units.degreesToRadians(alliance == Alliance.Blue ? 0.0 : 180.0))));

    } else {
      double flipped = -1.0; // so that intake is forward in driver oriented mode
      double chassisSpeedX = translation.getX() * flipped;
      double chassisSpeedY = translation.getY() * flipped;
      if (m_swerveActionFuntion.get() == SwerveAction.AIMNOTE) {
        chassisSpeedX += 0;
      }
      chassisSpeeds = new ChassisSpeeds(chassisSpeedX, chassisSpeedY, angularVelocity);
    }

    // chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    SwerveModuleState[] moduleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    if (WCLogger.isEnabled) {
      Logger.recordOutput(this.getClass().getSimpleName() + "/ExpectedModuleStates", moduleStates);
      Logger.recordOutput(this.getClass().getSimpleName() + "/ExpectedVelocity", linearVelocity);
      Logger.recordOutput(
          this.getClass().getSimpleName() + "/ExpectedAngularVelocity", angularVelocity);
    }
    m_swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean aimbotPIDAtSetpoint() {
    return m_aimBotPID.atSetpoint();
  }
}
