package frc.robot.util.robotswitch;

import frc.robot.constants.RobotConstants.RobotConfigConstants;
import frc.robot.util.robotswitch.RobotSwitchManager.RobotType;
import java.util.HashMap;

public class RobotSwitch<T> {
  public T defaultValue;
  public HashMap<RobotType, T> robotHashMapType = new HashMap<RobotType, T>();

  public RobotSwitch(T defaultValue) {
    this.defaultValue = defaultValue;
  }

  public RobotSwitch<T> addValue(RobotType robot, T value) {
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
