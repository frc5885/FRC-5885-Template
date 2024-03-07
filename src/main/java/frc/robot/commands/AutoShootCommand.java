// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.base.subsystems.PoseEstimator.PhotonVisionSystem;
import frc.robot.base.subsystems.PoseEstimator.SwervePoseEstimator;
import frc.robot.base.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootCommand extends ParallelDeadlineGroup {
	
	private final FeederSubsystem m_feederSubsystem;
	private final WristSubsystem m_wristSubsystem;
	private final SwerveDriveSubsystem m_swerveDriveSubsystem;
	private final SwervePoseEstimator m_poseEstimator;
	private final PhotonVisionSystem m_photonVision;
	
	public AutoShootCommand(Robot robot, FeederSubsystem feederSubsystem, WristSubsystem wristSubsystem, SwerveDriveSubsystem swerveDriveSubsystem, 
							SwervePoseEstimator poseEstimator, PhotonVisionSystem photonVision) {
		
		// the group will stop running when the super command finishes
		// wait 2 seconds (to aim) then run feeder for 2 seconds, shooter should already be spinning
		super(new SequentialCommandGroup(
		new InstantCommand(() -> robot.setAimBotting(true)),
		new WaitCommand(2),
		new InstantCommand(() -> feederSubsystem.intake()),
		new WaitCommand(2),
		new InstantCommand(() -> feederSubsystem.stop()),
		new InstantCommand(() -> robot.setAimBotting(false))
		));
		
		m_feederSubsystem = feederSubsystem;
		m_wristSubsystem = wristSubsystem;
		m_swerveDriveSubsystem = swerveDriveSubsystem;
		m_poseEstimator = poseEstimator;
		m_photonVision = photonVision;
		
		// these will stay running as long as the super command is running
		addCommands(
		new AimSwerveToTargetCommand(m_swerveDriveSubsystem, m_poseEstimator, m_photonVision)
		// wrist aimer will go here
		);
		// swerve drive and wrist are required by their respective commands, don't include them here
		addRequirements(m_feederSubsystem);
	}
}
