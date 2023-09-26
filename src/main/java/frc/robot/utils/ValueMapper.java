package frc.robot.utils;

import java.util.Map;
import java.util.TreeMap;

/**
 * A utility class for mapping values using various smoothing methods. The class uses a TreeMap to
 * store the mapped values and provides methods to add new values to the map and retrieve the mapped
 * value for a given input using the set smoothing method. The smoothing method can be set using the
 * SMOOTHING_METHOD enum, which includes various easing functions. The class also includes methods
 * to set the smoothing method and construct a new ValueMapper object with a specified smoothing
 * method.
 *
 * @see <a href="https://easings.net/">Easing Functions</a>
 * @see <a href="https://www.desmos.com/calculator/retkcgaqas">Easing Functions Graph</a>
 */
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
   * This class provides methods for mapping input values to output values using different smoothing
   * methods. The default smoothing method is linear.
   *
   * @constructor Initializes the smoothing method to linear.
   */
  public ValueMapper() {
    m_smoothing_method = SMOOTHING_METHOD.LINEAR;
  }

  /**
   * Constructs a ValueMapper object with the specified smoothing method.
   *
   * @param smoothing_method the smoothing method to be used by the ValueMapper object
   */
  public ValueMapper(SMOOTHING_METHOD smoothing_method) {
    m_smoothing_method = smoothing_method;
  }

  /**
   * Inserts a key-value pair into the ValueMapper.
   *
   * @param x the key to be inserted
   * @param y the value to be inserted
   */
  public void put(double x, double y) {
    m_map.put(x, y);
  }

  /**
   * Maps the given x and y arrays to a HashMap.
   *
   * @param x the array of x values
   * @param y the array of y values
   * @throws IllegalArgumentException if the length of x and y arrays are not equal
   */
  public void put(double[] x, double[] y) {
    if (x.length != y.length) throw new IllegalArgumentException("Arrays must be the same length");

    for (int i = 0; i < x.length; i++) {
      m_map.put(x[i], y[i]);
    }
  }

  /**
   * Maps a value to a corresponding output value using a set of input-output pairs and a smoothing
   * function.
   *
   * @param x the input value to be mapped
   * @return the output value corresponding to the input value
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

  /**
   * Sets the smoothing method to be used by the ValueMapper.
   *
   * @param smoothing_method the smoothing method to be used
   */
  public void setSmoothingMethod(SMOOTHING_METHOD smoothing_method) {
    m_smoothing_method = smoothing_method;
  }
}
