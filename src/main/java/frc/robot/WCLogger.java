package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.base.subsystems.SubsystemAction;

public class WCLogger {
  public static final boolean isEnabled = true;

  public static boolean putBoolean(Object object, String key, boolean value) {
    if (!isEnabled) {
      return true;
    }
    return SmartDashboard.putBoolean(object.getClass().getSimpleName() + "/" + key, value);
  }

  public static boolean getBoolean(String key, boolean defaultValue) {
    if (!isEnabled) {
      return defaultValue;
    }
    return SmartDashboard.getBoolean(key, defaultValue);
  }

  public static boolean putNumber(Object object, String key, double value) {
    if (!isEnabled) {
      return true;
    }
    return SmartDashboard.putNumber(object.getClass().getSimpleName() + "/" + key, value);
  }

  public static double getNumber(String key, double defaultValue) {
    if (!isEnabled) {
      return defaultValue;
    }
    return SmartDashboard.getNumber(key, defaultValue);
  }

  public static boolean putString(Object object, String key, String value) {
    if (!isEnabled) {
      return true;
    }
    return SmartDashboard.putString(
        object.getClass().getSimpleName() + "/" + key, value == null ? "null" : value);
  }

  public static String getString(String key, String defaultValue) {
    if (!isEnabled) {
      return defaultValue;
    }
    return SmartDashboard.getString(key, defaultValue);
  }

  public static boolean putAction(Object object, String key, SubsystemAction action) {
    if (!isEnabled) {
      return true;
    }
    return SmartDashboard.putString(
        object.getClass().getSimpleName() + "/" + key, action == null ? "null" : action.toString());
  }

  public static void putData(Object object, String key, Sendable data) {
    if (!isEnabled) {
      return;
    }
    SmartDashboard.putData(object.getClass().getSimpleName() + "/" + key, data);
  }

  public static Sendable getData(String key) {
    return SmartDashboard.getData(key);
  }
}
