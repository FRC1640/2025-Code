package frc.robot.util.tools.WPICal;

import frc.robot.constants.RobotConstants.AprilTagPositionSettings;
import java.util.HashMap;

public class AprilTagPositionSwitcher<T> {
  public enum AprilTagSetting {
    Hardcoded,
    WPILibWelded,
    WPILibAndyMark,
    WPICal,
  }

  public T defaultValue;
  public HashMap<AprilTagSetting, T> aprilTagSettingHash = new HashMap<AprilTagSetting, T>();

  public AprilTagPositionSwitcher(T defaultValue) {
    this.defaultValue = defaultValue;
  }

  public AprilTagPositionSwitcher<T> addValue(AprilTagSetting robot, T value) {
    aprilTagSettingHash.put(robot, value);
    return this;
  }

  public T get() {
    if (aprilTagSettingHash.containsKey(AprilTagPositionSettings.fieldPositionType)) {
      return aprilTagSettingHash.get(AprilTagPositionSettings.fieldPositionType);
    } else {
      return defaultValue;
    }
  }
}
