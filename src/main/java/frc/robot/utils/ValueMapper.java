package frc.robot.utils;

import java.util.Map;
import java.util.TreeMap;

public class ValueMapper {

  // Sets the easing function to be used when mapping the values. Higher the power the "stepper" the
  // smoothing method is used. I would recommend checking out the link below to see how each
  // function behaves. Typically 'LINEAR' or 'EASE_SINE' work the best by default. Higher power
  // smoothing functions will require more computing time.
  // https://easings.net/
  // https://www.desmos.com/calculator/retkcgaqas
  public enum SMOOTHING_METHOD {
    LINEAR,
    FLOOR,
    CEIL,
    ROUND,
    EASE_SINE,
    EASE_QUAD,
    EASE_CUBIC,
    EASE_QUART,
    EASE_EXPO
  }

  private TreeMap<Double, Double> m_map = new TreeMap<Double, Double>();
  private SMOOTHING_METHOD m_smoothing_method = SMOOTHING_METHOD.LINEAR;

  /**
   * Define a new ValueMapper, typically used for mapping system estimates to physical values (eg.
   * distance -> motor power).
   */
  public ValueMapper() {
    m_smoothing_method = SMOOTHING_METHOD.LINEAR;
  }

  /**
   * Define a new ValueMapper, typically used for mapping system estimates to physical values (eg.
   * distance -> motor power).
   *
   * @param smoothing_method Smoothing method to be used.
   */
  public ValueMapper(SMOOTHING_METHOD smoothing_method) {
    m_smoothing_method = smoothing_method;
  }

  /**
   * Add new value to the map
   *
   * @param x X value to map to Y
   * @param y Y value to map
   */
  public void put(double x, double y) {
    m_map.put(x, y);
  }

  /**
   * Get the mapped value from a given input
   *
   * @param x Input value
   * @return Mapped value with set smoothing method
   */
  public double get(double x) {
    Map.Entry<Double, Double> lower_entry = m_map.floorEntry(x);
    Map.Entry<Double, Double> upper_entry = m_map.ceilingEntry(x);

    if (lower_entry == null) return m_map.firstEntry().getValue();
    if (upper_entry == null) return m_map.lastEntry().getValue();

    // Normalized x distance
    double p = (x - lower_entry.getKey()) / (upper_entry.getKey() - lower_entry.getKey());
    double y_dif = upper_entry.getValue() - lower_entry.getValue();

    // Smoothing functions
    // Functions from https://easings.net/
    switch (m_smoothing_method) {
      case LINEAR:
        return p * y_dif + lower_entry.getValue();
      case FLOOR:
        return lower_entry.getValue();
      case CEIL:
        return upper_entry.getValue();
      case ROUND:
        return (p > 0.5f) ? upper_entry.getValue() : lower_entry.getValue();
      case EASE_SINE:
        return (double) (-(Math.cos(Math.PI * p) - 1.0) / 2.0) * y_dif + lower_entry.getValue();
      case EASE_QUAD:
        return (double) (p < 0.5 ? 2.0 * p * p : 1.0 - Math.pow(-2.0 * p + 2.0, 2.0) / 2.0) * y_dif
            + lower_entry.getValue();
      case EASE_CUBIC:
        return (double) (p < 0.5 ? 4.0 * p * p * p : 1.0 - Math.pow(-2.0 * p + 2.0, 3.0) / 2.0)
                * y_dif
            + lower_entry.getValue();
      case EASE_QUART:
        return (double) (p < 0.5 ? 8.0 * p * p * p * p : 1.0 - Math.pow(-2.0 * p + 2.0, 4.0) / 2.0)
                * y_dif
            + lower_entry.getValue();
      case EASE_EXPO:
        return (double)
                    (p == 0.0
                        ? 0.0
                        : p == 1.0
                            ? 1.0
                            : p < 0.5
                                ? Math.pow(2.0, 20.0 * p - 10.0) / 2.0
                                : (2.0 - Math.pow(2.0, -20.0 * p + 10.0)) / 2.0)
                * y_dif
            + lower_entry.getValue();
      default: // How did we get here?
        break;
    }

    return 0;
  }

  public void setSmoothingMethod(SMOOTHING_METHOD smoothing_method) {
    m_smoothing_method = smoothing_method;
  }
}
