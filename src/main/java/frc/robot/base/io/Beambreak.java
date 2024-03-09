package frc.robot.base.io;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Beambreak extends DigitalInput {

  // private LEDSubsystem m_ledSubsystem;

  public Beambreak() {
    super(Constants.kBeambreak);
    // m_ledSubsystem = ledSubsystem;
  }

  public Boolean isBroken() {
    // m_ledSubsystem.setSolid(0, 255, 0);
    return get();
  }

  public Boolean isOpen() {
    // m_ledSubsystem.setSolid(255, 0, 0);
    return !get();
  }
}
