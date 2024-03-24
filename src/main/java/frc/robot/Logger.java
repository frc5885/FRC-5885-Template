package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger {
  public static final boolean isDebugMode = false;

  public class SmartDashboard {
    public static boolean putBoolean(String key, boolean value) {
      if (!isDebugMode) {
        return true;
      }
      return SmartDashboard.putBoolean(key, value);
    }

    public static boolean getBoolean(String key, boolean defaultValue) {
      if (!isDebugMode) {
        return defaultValue;
      }
      return SmartDashboard.getBoolean(key, defaultValue);
    }

    public static boolean putNumber(String key, double value) {
      if (!isDebugMode) {
        return true;
      }
      return SmartDashboard.putNumber(key, value);
    }

    public static double getNumber(String key, double defaultValue) {
      if (!isDebugMode) {
        return defaultValue;
      }
      return SmartDashboard.getNumber(key, defaultValue);
    }

    public static boolean putString(String key, String value) {
      if (!isDebugMode) {
        return true;
      }
      return SmartDashboard.putString(key, value);
    }

    public static String getString(String key, String defaultValue) {
      if (!isDebugMode) {
        return defaultValue;
      }
      return SmartDashboard.getString(key, defaultValue);
    }

    public static void putData(String key, Sendable data) {
      if (!isDebugMode) {
        return;
      }
      SmartDashboard.putData(key, data);
    }

    public static Sendable getData(String key) {
      return SmartDashboard.getData(key);
    }
  }

  static void logAdvantageScope() {}
}
