package frc.robot.subsystems.TankDriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public class TankDriveSim implements TankDriveIO {

  private DifferentialDrivetrainSim sim =
      DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

  public TankDriveSim() {}

  @Override
  public void updateInputs(TankDriveIOInputs inputs) {
    sim.update(0.02);
    inputs.leftPositionMeters = sim.getLeftPositionMeters();
    inputs.leftVelocityMetersPerSec = sim.getLeftVelocityMetersPerSecond();
    inputs.rightPositionMeters = sim.getRightPositionMeters();
    inputs.rightVelocityMetersPerSec = sim.getRightVelocityMetersPerSecond();
    inputs.gyroYawRad = sim.getHeading().getRadians();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    sim.setInputs(MathUtil.clamp(leftVolts, -12.0, 12.0), MathUtil.clamp(rightVolts, -12.0, 12.0));
  }

  @Override
  public void resetGyro() {
    return;
  }

  @Override
  public void resetEncoders() {
    return;
  }
}
