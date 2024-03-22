// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.base;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.base.subsystems.swerve.SwerveAction;
import java.io.File;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class RobotSystem extends LoggedRobot {
  private Command m_autonomousCommand;
  private Robot m_robotContainer;

  // private LEDSubsystem m_ledSubsystem;

  @Override
  public void robotInit() {

    // m_ledSubsystem = new LEDSubsystem();
    // m_LedSubsystem.setLedRainbow();

    // Copied from advtangekit examples
    // Record metadata
    // Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    // Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    // Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    // Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    // Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    // switch (BuildConstants.DIRTY) {
    // case ;:
    // Logger.recordMetadata("GitDirty", "All changes committed");
    // break;
    // case 1:
    // Logger.recordMetadata("GitDirty", "Uncomitted changes");
    // break;
    // default:
    // Logger.recordMetadata("GitDirty", "Unknown");
    // break;
    // }

    // Copied from advtangekit examples
    // Set up data receivers & replay source
    if (RobotBase.isReal()) {
      // Running on a real robot, log to a USB stick

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
    enableLiveWindowInTest(true);
    m_robotContainer = new Robot();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.m_shooterSubsystem.spinFastClose();
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
    m_robotContainer.setSwerveAction(SwerveAction.DEFAULT);
    m_robotContainer.m_shooterSubsystem.stop();
    m_robotContainer.m_intakeSubsystem.stop();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
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
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
