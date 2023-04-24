package frc.robot.utils;

// Smoothing functions for normilized values (0 -> 1)
// All of these functions are expected to be used with additional code such as the value mapper.
// Code from https://easings.net/
public class SmoothingFunctions {

  public static double norm_linear(double x) {
    return x;
  }

  public static double norm_floor(double x) {
    return Math.floor(x);
  }

  public static double norm_ceil(double x) {
    return Math.ceil(x);
  }

  public static double norm_round(double x) {
    return Math.round(x);
  }

  public static double norm_ease_sign(double x) {
    return (-(Math.cos(Math.PI * x) - 1.0) / 2.0);
  }

  public static double norm_ease_quad(double x) {
    return (x < 0.5 ? 2.0 * x * x : 1.0 - Math.pow(-2.0 * x + 2.0, 2.0) / 2.0);
  }

  public static double norm_ease_cubic(double x) {
    return (x < 0.5 ? 4.0 * x * x * x : 1.0 - Math.pow(-2.0 * x + 2.0, 3.0) / 2.0);
  }

  public static double norm_ease_quart(double x) {
    return (x < 0.5 ? 8.0 * x * x * x * x : 1.0 - Math.pow(-2.0 * x + 2.0, 4.0) / 2.0);
  }

  public static double norm_ease_expo(double x) {
    return (x == 0.0
        ? 0.0
        : x == 1.0
            ? 1.0
            : x < 0.5
                ? Math.pow(2.0, 20.0 * x - 10.0) / 2.0
                : (2.0 - Math.pow(2.0, -20.0 * x + 10.0)) / 2.0);
  }
}
