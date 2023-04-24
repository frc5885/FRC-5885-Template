package frc.utils;

import edu.wpi.first.wpilibj.XboxController;
import java.util.function.Function;

public class ImprovedXboxController extends XboxController {

  Function<Double, Double> m_left_trig_map_func = SmoothingFunctions::norm_linear;
  Function<Double, Double> m_right_trig_map_func = SmoothingFunctions::norm_linear;
  Function<Double, Double> m_left_map_func = SmoothingFunctions::norm_linear;
  Function<Double, Double> m_right_map_func = SmoothingFunctions::norm_linear;

  double m_deadzone = 0.005;

  public ImprovedXboxController(int port) {
    super(port);
  }

  // Honestly no idea what this does
  // @Override
  // public double getRightTriggerAxis() { }

  @Override
  public double getLeftX() {
    if (Math.abs(super.getLeftX()) < m_deadzone) return 0;
    return m_left_map_func.apply(super.getLeftX());
  }

  @Override
  public double getLeftY() {
    if (Math.abs(super.getLeftY()) < m_deadzone) return 0;
    return m_left_map_func.apply(super.getLeftY());
  }

  @Override
  public double getLeftTriggerAxis() {
    if (Math.abs(super.getLeftTriggerAxis()) < m_deadzone) return 0;
    return m_left_trig_map_func.apply(super.getLeftTriggerAxis());
  }

  @Override
  public double getRightX() {
    if (Math.abs(super.getRightX()) < m_deadzone) return 0;
    return m_right_map_func.apply(super.getRightX());
  }

  @Override
  public double getRightY() {
    if (Math.abs(super.getRightY()) < m_deadzone) return 0;
    return m_right_map_func.apply(super.getRightY());
  }

  @Override
  public double getRightTriggerAxis() {
    if (Math.abs(super.getRightTriggerAxis()) < m_deadzone) return 0;
    return m_right_trig_map_func.apply(super.getRightTriggerAxis());
  }

  /**
   * Change the deadzone of the controller
   *
   * @param deadzone Values reported under this will return zero.
   */
  public void setDeadzone(double deadzone) {
    m_deadzone = deadzone;
  }

  /**
   * Sets the mapping function for the controller (in case you don't want linear values)
   *
   * @param func Accepts any function that consumes a float and returns a float
   */
  public void setMappingFunction(Function<Double, Double> func) {
    m_left_map_func = func;
    m_right_map_func = func;
  }

  /**
   * Sets the mapping function for the controller (in case you don't want linear values) for only
   * the left joystick.
   *
   * @param func Accepts any function that consumes a float and returns a float
   */
  public void setLeftMappingFunction(Function<Double, Double> func) {
    m_left_map_func = func;
  }

  /**
   * Sets the mapping function for the controller (in case you don't want linear values) for only
   * the right joystick.
   *
   * @param func Accepts any function that consumes a float and returns a float
   */
  public void setRightMappingFunction(Function<Double, Double> func) {
    m_right_map_func = func;
  }

  /**
   * Sets the mapping function for the controller (in case you don't want linear values) for only
   * the right trigger.
   *
   * @param func Accepts any function that consumes a float and returns a float
   */
  public void setRightTriggerMappingFunction(Function<Double, Double> func) {
    m_right_trig_map_func = func;
  }

  /**
   * Sets the mapping function for the controller (in case you don't want linear values) for only
   * the left trigger.
   *
   * @param func Accepts any function that consumes a float and returns a float
   */
  public void setLeftTriggerMappingFunction(Function<Double, Double> func) {
    m_left_trig_map_func = func;
  }
}
