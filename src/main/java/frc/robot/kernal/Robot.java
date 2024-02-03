// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kernal;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {

    // Copied from advtangekit examples
    // Record metadata
    // Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    // Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    // Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    // Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    // Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    // switch (BuildConstants.DIRTY) {
    //   case 0:
    //     Logger.recordMetadata("GitDirty", "All changes committed");
    //     break;
    //   case 1:
    //     Logger.recordMetadata("GitDirty", "Uncomitted changes");
    //     break;
    //   default:
    //     Logger.recordMetadata("GitDirty", "Unknown");
    //     break;
    // }

    // Copied from advtangekit examples
    // Set up data receivers & replay source
    if (RobotBase.isReal()) {
      // Running on a real robot, log to a USB stick
      // TODO: Wait for fix
      // Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      // Running a physics simulator, log to local folder
      Logger.addDataReceiver(new WPILOGWriter("logs/"));
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();
    m_robotContainer = new RobotContainer();
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
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

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
