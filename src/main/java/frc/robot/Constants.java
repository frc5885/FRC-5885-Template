// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    // Motor ports
    public static final int kLeftFrontMotorPort = 1;
    public static final int kLeftRearMotorPort = 2;
    public static final boolean kLeftMotorsReversed = false;

    public static final int kRightFrontMotorPort = 3;
    public static final int kRightRearMotorPort = 4;
    public static final boolean kRightMotorsReversed = true;

    // Encoders
    public static final int kLeftEncoderPortA = 0;
    public static final int kLeftEncoderPortB = 1;
    public static final boolean kLeftEncoderReversed = false;

    public static final int kRightEncoderPortA = 2;
    public static final int kRightEncoderPortB = 3;
    public static final boolean kRightEncoderReversed = true;

    public static final int kEncoderResolution = 400;

    // Physical contraints
    public static final double kTrackWidthMeters = 0.7874;
    public static final double kWheelRadiusMeters = 0.1524; // 6" -> 0.1524m
    public static final double kEncoderGearboxRatio =
        1; // Toughbox mini has a 1:1 output for encoders
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelRadiusMeters * Math.PI) / (double) kEncoderResolution;

    public static final double ksVolts = 0.89792;
    public static final double kvVoltSecondsPerMeter = 2.2403;
    public static final double kaVoltSecondsSquaredPerMeter = 0.60985;

    public static final double kPDriveVel = 3.2178;

    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
