// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TuningCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDrive;
import java.util.LinkedList;
import java.util.List;

public class SwerveSolveFeedForward extends Command {

  private final SwerveDrive m_swerveSubsystem;

  private final Timer m_timer = new Timer();

  private final double m_delay = 3.0;
  private final double m_rampRate = 0.025;
  private final double m_minimumWaitTime = 1;

  private final List<Double> m_xVelocityMetersPerSecond = new LinkedList<Double>();
  private final List<Double> m_yVoltage = new LinkedList<Double>();

  private double m_currentVoltage = 0.0;
  private double m_previousVelocityMetersPerSecond = 0.0;
  private double m_lastSetTime = 0.0;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveSolveFeedForward(SwerveDrive swerveDrive) {
    m_swerveSubsystem = swerveDrive;

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_swerveSubsystem.resetGyro();

    m_xVelocityMetersPerSecond.clear();
    m_yVoltage.clear();
    m_currentVoltage = 0.0;
    m_previousVelocityMetersPerSecond = 0.0;
    m_lastSetTime = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Align wheels
    m_swerveSubsystem.setModulesAngle(0);
    double currentVelocityMetersPerSecond = m_swerveSubsystem.getChassisSpeeds().vxMetersPerSecond;

    if (m_timer.get() < m_delay) {
      return;
    } else {
      m_swerveSubsystem.setVoltage(m_currentVoltage);

      if ((currentVelocityMetersPerSecond - m_previousVelocityMetersPerSecond) / 0.02 <= 0.0005
          && (m_timer.get() - m_lastSetTime) >= m_minimumWaitTime) {
        m_xVelocityMetersPerSecond.add(currentVelocityMetersPerSecond);
        m_yVoltage.add(m_currentVoltage);
        m_lastSetTime = m_timer.get();

        m_currentVoltage += m_rampRate;

        System.out.printf("Steady state reached, increasing voltage to %.5f\n", m_currentVoltage);
      }
    }

    m_previousVelocityMetersPerSecond = currentVelocityMetersPerSecond;
  }

  public double[] calculateLeastSquares(List<Double> x, List<Double> y) {
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumXX = 0.0;

    for (int i = 0; i != x.size(); ++i) {
      sumX += x.get(i);
      sumY += y.get(i);
      sumXY += x.get(i) * y.get(i);
      sumXX += x.get(i) * x.get(i);
    }

    double slope = (x.size() * sumXY - sumX * sumY) / (x.size() * sumXX - sumX * sumX);
    double intercept = (sumY - slope * sumX) / x.size();

    return new double[] {slope, intercept};
  }

  public double calculateR2(List<Double> actual, List<Double> predicted) {
    double sumActual = 0.0;
    double sumPredicted = 0.0;
    double sumSquaredActual = 0.0;
    double sumSquaredPredicted = 0.0;
    double sumProduct = 0.0;
    int n = actual.size();

    for (int i = 0; i < n; i++) {
      double actualValue = actual.get(i);
      double predictedValue = predicted.get(i);
      sumActual += actualValue;
      sumPredicted += predictedValue;
      sumSquaredActual += actualValue * actualValue;
      sumSquaredPredicted += predictedValue * predictedValue;
      sumProduct += actualValue * predictedValue;
    }

    double numerator = n * sumProduct - sumActual * sumPredicted;
    double denominator =
        Math.sqrt(
            (n * sumSquaredActual - sumActual * sumActual)
                * (n * sumSquaredPredicted - sumPredicted * sumPredicted));
    double r2 = numerator / denominator;
    return r2 * r2;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stop();

    // If we don't have enough data, don't try to calculate anything
    if (m_xVelocityMetersPerSecond.size() < 3) {
      System.out.println("ERROR: Not enough data to calculate feed forward, exiting");
      return;
    }

    // Calculate the derivitive and find the point where the slope is >= 1
    // This is an arbitrary value, but it should be greater then 1
    int startPoint = 0;
    for (int i = 0; i != m_xVelocityMetersPerSecond.size() - 1; ++i) {
      double deriv =
          (m_yVoltage.get(i + 1) - m_yVoltage.get(i))
              / (m_xVelocityMetersPerSecond.get(i + 1) - m_xVelocityMetersPerSecond.get(i));
      if (deriv >= 1 && !Double.isInfinite(deriv) && !Double.isNaN(deriv)) {
        startPoint = i;
        System.out.println("Found start point at " + startPoint);
        break;
      }
    }

    if (startPoint == 0) {
      System.out.println(
          "WARNING: Could not find start point, using all data. Result may be inaccurate.");
    }

    // Delete everything before the start point
    for (int i = 0; i != startPoint; ++i) {
      m_xVelocityMetersPerSecond.remove(0);
      m_yVoltage.remove(0);
    }

    double[] rslt = calculateLeastSquares(m_xVelocityMetersPerSecond, m_yVoltage);

    System.out.printf("kS: %.8f\n", rslt[1]);
    System.out.printf("kV: %.8f\n", rslt[0]);

    // Calculate predicted values
    List<Double> predicted = new LinkedList<Double>();
    for (int i = 0; i != m_xVelocityMetersPerSecond.size(); ++i) {
      predicted.add(rslt[0] * m_xVelocityMetersPerSecond.get(i) + rslt[1]);
    }

    System.out.printf("R2: %.8f\n", calculateR2(m_yVoltage, predicted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
