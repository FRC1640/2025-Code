package frc.robot.util.robotswitch;

import frc.robot.util.robotswitch.RobotSwitchManager.RobotType;

public class RobotTypeParm<T> {
  public T value;
  public RobotType robotType;

  public RobotTypeParm(RobotType roboType, T val) {
    value = val;
    robotType = roboType;
  }
}
