package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.base.io.Beambreak;
import java.util.function.Supplier;

public class LEDSubsystem extends SubsystemBase {

  public enum LEDMode {
    OFF,
    RAINBOW,
    SOLID,
    TELEOP,
    AUTO
  }

  private AddressableLED m_led;
  private LEDMode m_mode;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private int m_r;
  private int m_g;
  private int m_b;

  Beambreak m_beambreak;
  Supplier<Boolean> m_isAimbottingFunction;

  public LEDSubsystem(Beambreak beambreak, Supplier<Boolean> isAimbottingFunction) {
    m_led = new AddressableLED(Constants.kLED);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_mode = LEDMode.OFF;
    m_led.start();
    m_rainbowFirstPixelHue = 0;

    m_beambreak = beambreak;
    m_isAimbottingFunction = isAimbottingFunction;
  }

  @Override
  public void periodic() {
    switch (m_mode) {
      case OFF:
        setLedColor(0, 0, 0);
        break;
      case RAINBOW:
        updateLedRainbow();
        break;
      case SOLID:
        setLedColor(m_r, m_g, m_b);
        break;
      case TELEOP:
        if (m_beambreak.isBroken()) {
          if (m_isAimbottingFunction.get()) {
            setLedColor(0, 255, 0);
          } else {
            setLedColor(255, 255, 255);
          }
        } else {
          setLedColor(0, 0, 0);
        }
        break;
      case AUTO:
        // TODO AUTO LED CODE
        updateLedRainbow();
        break;
    }
  }

  public void setLedColor(int r, int g, int b) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }

  // (0°, 100%, 100%)
  public void updateLedRainbow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + +(i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_rainbowFirstPixelHue += 3;
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
}
