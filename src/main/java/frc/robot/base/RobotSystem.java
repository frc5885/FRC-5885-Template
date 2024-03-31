// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.base;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AprilTagCameraConstants;
import frc.robot.Robot;
import frc.robot.WCLogger;
import frc.robot.base.modules.swerve.SwerveConstants;
import frc.robot.base.subsystems.swerve.SwerveAction;
import frc.robot.commands.StowWristCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.test.*;
import frc.robot.subsystems.ShooterSubsystem.RobotMode;
import java.io.File;
import java.util.Map;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class RobotSystem extends LoggedRobot {
  private Command m_autonomousCommand;
  private Robot m_robotContainer;

  @Override
  public void robotInit() {
    if (WCLogger.isEnabled) {
      if (RobotBase.isReal()) {
        boolean found_thumbdrive = false;
        File directory = new File("/media/");
        if (directory.isDirectory()) {
          File[] subdirectories = directory.listFiles(File::isDirectory);
          if (subdirectories != null) {
            for (File subdirectory : subdirectories) {
              File[] files = subdirectory.listFiles();
              if (files != null) {
                for (File file : files) {
                  if (file.getName().equals("start_logging")) {
                    // Do something when the file is found
                    Logger.addDataReceiver(new WPILOGWriter(file.getParent()));
                    System.out.println("Writting to USB! @" + file.getParent());
                    found_thumbdrive = true;
                    break;
                  }
                }
              }
            }
          }
        }
        Logger.addDataReceiver(new NT4Publisher());

        if (!found_thumbdrive) {
          System.out.println("Not logging to usb!");
        }

        Logger.addDataReceiver(new NT4Publisher());
      } else {
        // Running a physics simulator, log to local folder
        Logger.addDataReceiver(new WPILOGWriter("logs/"));
        Logger.addDataReceiver(new NT4Publisher());
      }

      Logger.start();
    }
    m_robotContainer = new Robot();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (WCLogger.isEnabled) {
      long total = Runtime.getRuntime().totalMemory();
      long free = Runtime.getRuntime().freeMemory();
      WCLogger.putNumber(this, "RAM/Total", total);
      WCLogger.putNumber(this, "RAM/Max", Runtime.getRuntime().maxMemory());
      WCLogger.putNumber(this, "RAM/Free", Runtime.getRuntime().freeMemory());
      WCLogger.putNumber(this, "RAM/Used", total - free);
    }
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setLEDsRainbow();
  } 

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putBoolean(
        "AUTO SELECTED",
        !m_robotContainer.m_simplePathPlanner.getSelectedAuto().getName().equals("InstantCommand"));
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    setCameraOffset();
    m_robotContainer.m_shooterSubsystem.robotMode = RobotMode.AUTO;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    double distanceToTarget =
        m_robotContainer.m_photonVision.getDistanceToTarget(
            m_robotContainer.m_swervePoseEstimator.getPose(),
            m_robotContainer.m_photonVision.getTargetID());
    if (distanceToTarget >= 3.1) {
      m_robotContainer.m_shooterSubsystem.spinFastFar();
    } else {
      m_robotContainer.m_shooterSubsystem.spinFastClose();
    }
    if (m_robotContainer.m_beambreak.isOpen()) {
      m_robotContainer.m_intakeSubsystem.intake();
      m_robotContainer.m_feederSubsystem.intake();
    } else {
      m_robotContainer.m_intakeSubsystem.stop();
      if (m_robotContainer.m_wristSubsystem.isStowed()) {
        m_robotContainer.m_feederSubsystem.stop();
      }
    }
  }

  @Override
  public void autonomousExit() {
    // m_robotContainer.setAimBotting(false);
    m_robotContainer.m_wristSubsystem.stop();
    m_robotContainer.m_feederSubsystem.stop();
    m_robotContainer.m_intakeSubsystem.stop();
    m_robotContainer.m_shooterSubsystem.stop();
    m_robotContainer.setSwerveAction(SwerveAction.DEFAULT);
  }

  @Override
  public void teleopInit() {
    setCameraOffset();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_shooterSubsystem.robotMode = RobotMode.TELEOP;
    m_robotContainer.m_wristSubsystem.stop();
    m_robotContainer.m_feederSubsystem.stop();
    m_robotContainer.m_intakeSubsystem.stop();
    m_robotContainer.m_shooterSubsystem.stop();
    m_robotContainer.setSwerveAction(SwerveAction.DEFAULT);
    new StowWristCommand(m_robotContainer.m_armSubsystem, m_robotContainer.m_wristSubsystem)
        .schedule();
    m_robotContainer.setLEDsTeleop();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    // m_robotContainer.setAimBotting(false);
    m_robotContainer.setSwerveAction(SwerveAction.DEFAULT);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    ShuffleboardTab testTab = Shuffleboard.getTab("Systems Test");

    // System
    testTab.add(
        "System.Test",
        new TestSystemCommand(
            m_robotContainer.m_swerveDrive,
            m_robotContainer.m_armSubsystem,
            m_robotContainer.m_wristSubsystem,
            m_robotContainer.m_shooterSubsystem,
            m_robotContainer.m_intakeSubsystem,
            m_robotContainer.m_feederSubsystem,
            m_robotContainer.m_ledSubsystem
        )
    );

    // Swerve
    testTab.add(
            "Swerve.Drive.Positive",
            new SequentialCommandGroup(
                new TestResetSwerveCommand(m_robotContainer.m_swerveDrive),
                new TestSwerveCommand(
                    m_robotContainer.m_swerveDrive,
                    TestSwerveCommand.Type.FORWARD,
                    m_robotContainer.m_ledSubsystem
                )
            )
        )
        .withWidget(BuiltInWidgets.kCommand);
    testTab.add(
            "Swerve.Drive.Negative",
            new SequentialCommandGroup(
              new TestResetSwerveCommand(m_robotContainer.m_swerveDrive),
              new TestSwerveCommand(
                  m_robotContainer.m_swerveDrive,
                  TestSwerveCommand.Type.BACKWARD,
                  m_robotContainer.m_ledSubsystem
              )
          )
        )
        .withWidget(BuiltInWidgets.kCommand);
    testTab.add(
            "Swerve.Rotate.Positive",
            new SequentialCommandGroup(
              new TestResetSwerveCommand(m_robotContainer.m_swerveDrive),
              new TestSwerveCommand(
                  m_robotContainer.m_swerveDrive,
                  TestSwerveCommand.Type.ROTATE_CLOCKWISE,
                  m_robotContainer.m_ledSubsystem
              )
            )
        )
        .withWidget(BuiltInWidgets.kCommand);
    testTab.add(
            "Swerve.Rotate.Negative",
            new SequentialCommandGroup(
              new TestResetSwerveCommand(m_robotContainer.m_swerveDrive),
              new TestSwerveCommand(
                  m_robotContainer.m_swerveDrive,
                  TestSwerveCommand.Type.ROTATE_COUNTER_CLOCKWISE,
                  m_robotContainer.m_ledSubsystem
              )
            )
        )
        .withWidget(BuiltInWidgets.kCommand);

    // Arm
    testTab
        .add(
            "Arm.Up",
            new TestArmUpCommand(
                m_robotContainer.m_armSubsystem,
                m_robotContainer.m_ledSubsystem
            )

        );
    testTab
        .add(
            "Arm.Down",
            new TestArmDownCommand(
                m_robotContainer.m_armSubsystem,
                m_robotContainer.m_ledSubsystem
            )

        );

    // Wrist
    testTab
        .add(
            "Wrist.Aim",
            new TestWristAimCommand(
                m_robotContainer.m_wristSubsystem,
                m_robotContainer.m_ledSubsystem
            )

        );
    testTab
        .add(
            "Wrist.Stow",
            new TestWristStowCommand(
                m_robotContainer.m_wristSubsystem,
                m_robotContainer.m_ledSubsystem
            )

        );

    // Shooters
    testTab
        .add(
            "Shooter.Velocity",
            new TestShooterCommand(
                m_robotContainer.m_shooterSubsystem,
                m_robotContainer.m_ledSubsystem
            )

        );

    // Intake
    testTab
        .add(
            "Intake.Intake",
            new TestIntakeCommand(
                m_robotContainer.m_intakeSubsystem,
                TestIntakeCommand.Type.INTAKE,
                m_robotContainer.m_ledSubsystem
            )

        );
    testTab
        .add(
            "Intake.Outtake",
            new TestIntakeCommand(
                m_robotContainer.m_intakeSubsystem,
                TestIntakeCommand.Type.OUTTAKE,
                m_robotContainer.m_ledSubsystem
            )

        );

    // Feeder
    testTab
        .add(
            "Feeder.Intake",
            new TestFeederCommand(
                m_robotContainer.m_feederSubsystem,
                TestFeederCommand.Type.OUTTAKE,
                m_robotContainer.m_ledSubsystem
            )

        );
    testTab
        .add(
            "Feeder.Outtake",
            new TestFeederCommand(
                m_robotContainer.m_feederSubsystem,
                TestFeederCommand.Type.OUTTAKE,
                m_robotContainer.m_ledSubsystem
            )

        );
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private void setCameraOffset() {
    Alliance alliance = DriverStation.getAlliance().get();
    if (alliance == Alliance.Blue) {
      m_robotContainer.m_photonVision.setRobotToCameraTransform(
          new Transform3d(
              new Translation3d(
                  AprilTagCameraConstants.ShooterBlue.kCameraPositionX,
                  AprilTagCameraConstants.ShooterBlue.kCameraPositonY,
                  AprilTagCameraConstants.ShooterBlue.kCameraPositionZ),
              new Rotation3d(
                  AprilTagCameraConstants.ShooterBlue.kCameraRoll,
                  AprilTagCameraConstants.ShooterBlue.kCameraPitch,
                  AprilTagCameraConstants.ShooterBlue.kCameraYaw)));
    } else if (alliance == Alliance.Red) {
      m_robotContainer.m_photonVision.setRobotToCameraTransform(
          new Transform3d(
              new Translation3d(
                  AprilTagCameraConstants.ShooterRed.kCameraPositionX,
                  AprilTagCameraConstants.ShooterRed.kCameraPositonY,
                  AprilTagCameraConstants.ShooterRed.kCameraPositionZ),
              new Rotation3d(
                  AprilTagCameraConstants.ShooterRed.kCameraRoll,
                  AprilTagCameraConstants.ShooterRed.kCameraPitch,
                  AprilTagCameraConstants.ShooterRed.kCameraYaw)));
    }
  }
}
