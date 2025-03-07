package frc.robot.util.tools;

import frc.robot.util.tools.RobotSwitchManager.RobotType;

public class RobotTypeParm<T> {
  public T value;
  public RobotType robotType;

  public RobotTypeParm(RobotType roboType, T val) {
    value = val;
    robotType = roboType;
  }
}
