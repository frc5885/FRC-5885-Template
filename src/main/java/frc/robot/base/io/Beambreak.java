package frc.robot.base.io;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Beambreak extends DigitalInput {

  public Beambreak() {
    super(Constants.kBeambreak);
  }

  public boolean isBroken() {
    return get();
  }

  public boolean isOpen() {
    return !get();
  }
}
