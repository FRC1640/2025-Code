package frc.robot.util.tools.WPICal;

import frc.robot.constants.RobotConstants.RobotConfigConstants;
import java.util.HashMap;

public class AprilTagPositionSwitcher<T> {
  public enum AprilTagSetting {
    Hardcoded,
    WPILib,
    WPICal;
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
    if (aprilTagSettingHash.containsKey(RobotConfigConstants.fieldPositionPositions)) {
      return aprilTagSettingHash.get(RobotConfigConstants.fieldPositionPositions);
    } else {
      return defaultValue;
    }
  }
}
