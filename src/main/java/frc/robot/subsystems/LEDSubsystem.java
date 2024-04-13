package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.base.io.Beambreak;
import frc.robot.base.subsystems.SubsystemAction;
import java.util.function.Supplier;

public class LEDSubsystem extends SubsystemBase {

  public enum LEDMode {
    OFF,
    RAINBOW,
    SOLID,
    TELEOP,
    AUTO,
    ERROR,
    TESTING,
    SUCCESS
  }

  // purple = 200, 0, 255

  private AddressableLED m_led;
  private LEDMode m_mode;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private int m_r;
  private int m_g;
  private int m_b;
  long m_lastFlash = 0;
  int m_flashRate = 175;
  int m_currentColor = 0;

  Beambreak m_beambreak;
  ShooterSubsystem m_shooterSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  Supplier<Boolean> m_isAimbottingFunction;
  Supplier<Double> m_distanceToTargetFunction;

  public LEDSubsystem(
      Beambreak beambreak,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      Supplier<Boolean> isAimbottingFunction,
      Supplier<Double> distanceToTargetFunction) {
    m_led = new AddressableLED(Constants.kLED);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_mode = LEDMode.OFF;
    m_led.start();
    m_rainbowFirstPixelHue = 0;

    m_beambreak = beambreak;
    m_isAimbottingFunction = isAimbottingFunction;
    m_shooterSubsystem = shooterSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_distanceToTargetFunction = distanceToTargetFunction;

    SmartDashboard.putNumber("LED/r", 0);
    SmartDashboard.putNumber("LED/g", 0);
    SmartDashboard.putNumber("LED/b", 0);
  }

  @Override
  public void periodic() {
    switch (m_mode) {
      case OFF:
        setLedColor(0, 0, 0);
        break;
      case RAINBOW:
        updateLedRainbow();
        // int r = (int) SmartDashboard.getNumber("LED/r", 0);
        // int g = (int) SmartDashboard.getNumber("LED/g", 0);
        // int b = (int) SmartDashboard.getNumber("LED/b", 0);
        // setLedColor(r, g, b);
        break;
      case SOLID:
        setLedColor(m_r, m_g, m_b);
        break;
      case TELEOP:
        if (m_beambreak.isBroken()) {
          if (m_isAimbottingFunction.get()) {
            if (m_distanceToTargetFunction.get() <= Constants.kShootCloseThreshold) {
              // solid green when close and aimbotting
              setLedColor(0, 255, 0);
            } else if (m_distanceToTargetFunction.get() <= Constants.kShootFarThreshold) {
              // solid yellow when far and aimbotting
              setLedColor(255, 60, 0);
            } else {
              // solid blue when passing
              setLedColor(0, 0, 255);
            }
          } else {
            if (m_distanceToTargetFunction.get() <= Constants.kShootCloseThreshold) {
              // flashing green when close
              flash(0, 255, 0);
            } else if (m_distanceToTargetFunction.get() <= Constants.kShootFarThreshold) {
              // flashing yellow when far
              flash(255, 60, 0);
            } else {
              // flashing blue when has note
              flash(0, 0, 255);
            }
          }
        } else if (m_intakeSubsystem.hasNote()) {
          // solid red when intake has note but beambreak is not broken
          setLedColor(255, 0, 0);
        } else if (m_intakeSubsystem.getSubsystemAction() == SubsystemAction.INTAKE) {
          // Solid white when intaking
          setLedColor(255, 255, 255);
        } else {
          // off when no note present
          setLedColor(0, 0, 0);
        }
        break;
      case AUTO:
        // TODO AUTO LED CODE
        updateLedRainbow();
        break;
      case ERROR:
        flash(255, 0, 0);
        break;
      case TESTING:
        flash(0, 0, 255);
        break;
      case SUCCESS:
        flash(0, 255, 0);
        break;
    }
  }

  private void flash(int r, int g, int b) {
    long now = System.currentTimeMillis();
    if (now - m_lastFlash > m_flashRate) {
      if (m_currentColor == 0) {
        setLedColor(r, g, b);
      } else {
        setLedColor(0, 0, 0);
      }
      m_lastFlash = now;
    }
  }

  public void setLedColor(int r, int g, int b) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, b, g);
    }
    m_led.setData(m_ledBuffer);
    m_currentColor = r + g + b;
  }

  // (0°, 100%, 100%)
  public void updateLedRainbow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + +(i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_rainbowFirstPixelHue += 1;
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }

  public void setRainbow() {
    m_mode = LEDMode.RAINBOW;
  }

  public void setSolid(int r, int g, int b) {
    m_r = r;
    // I think it is wired wrong but this works lol
    m_g = b;
    m_b = g;
    m_mode = LEDMode.SOLID;
  }

  public void setOff() {
    m_mode = LEDMode.OFF;
  }

  public void setTeleop() {
    m_mode = LEDMode.TELEOP;
  }

  public void setAuto() {
    m_mode = LEDMode.AUTO;
  }

  public void setMode(LEDMode mode) {
    m_mode = mode;
  }
}
