package frc.robot.util.tools;

import frc.robot.constants.RobotConstants.RobotConfigConstants;

public class RobotSwitch {
  public enum RobotType {
    Do25,
    Prime25,
  }

  public static <T> T robotTypeValue(T do25, T prime25) {
    switch (RobotConfigConstants.robotType) {
      case Do25:
        return do25;
      case Prime25:
        return prime25;
      default:
        return null;
    }
  }
}
