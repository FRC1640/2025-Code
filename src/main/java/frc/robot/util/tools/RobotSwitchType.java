package frc.robot.util.tools;

import frc.robot.constants.RobotConstants.RobotConfigConstants;
import frc.robot.util.tools.RobotSwitch.RobotType;
import java.util.HashMap;

public class RobotSwitchType<T> {
  public T defaultValue;
  public HashMap<RobotType, T> robotHashMapType;

  public RobotSwitchType(T defaultValue) {
    this.defaultValue = defaultValue;
  }

  public RobotSwitchType<T> addValue(RobotType robot, T value) {
    robotHashMapType.put(robot, value);
    return this;
  }

  public T get() {
    if (robotHashMapType.containsKey(RobotConfigConstants.robotType)) {
      return robotHashMapType.get(RobotConfigConstants.robotType);
    } else {
      return defaultValue;
    }
  }
}
