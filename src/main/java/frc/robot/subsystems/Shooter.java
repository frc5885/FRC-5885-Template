// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private CANSparkMax m_top;
  private CANSparkMax m_bottom;

  private double m_speed = 0.0;

  /** Creates a new Shooter. */
  public Shooter() {
    m_top = new CANSparkMax(50, MotorType.kBrushless);
    m_bottom = new CANSparkMax(51, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_top.setVoltage(m_speed*12);
    m_bottom.setVoltage(m_speed*12);
  }

  public void setSpeed(double speed) {
    m_speed = speed;
  }
}
