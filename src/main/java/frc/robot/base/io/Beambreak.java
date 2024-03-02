package frc.robot.base.io;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Beambreak extends DigitalInput {
  public Beambreak() {
    super(Constants.kBeambreak);
  }

  public Boolean isBroken() {
    return get();
  }

  public Boolean isOpen() {
    return !get();
  }
}
