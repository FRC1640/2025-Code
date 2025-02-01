package frc.robot.util.tools;

import frc.robot.constants.RobotConstants.RobotConfigConstants;

public class RobotSwitch {
  public enum RobotType {
    Duex25,
    Prime25,
  }

  public static <T> T robotTypeValue(T duex25, T prime25) {
    switch (RobotConfigConstants.robotType) {
      case Duex25:
        return duex25;
      case Prime25:
        return prime25;
      default:
        return null;
    }
  }
}
