package frc.robot.components;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Beambreak extends DigitalInput {
  public Beambreak() {
    super(Constants.kBeambreak);
  }

  public Boolean isBroken() {
    return !get();
  }
}
