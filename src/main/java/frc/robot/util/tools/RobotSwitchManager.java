package frc.robot.util.tools;

import frc.robot.constants.RobotConstants.RobotConfigConstants;

public class RobotSwitchManager {

  public enum RobotType {
    Deux25,
    Prime25,
    Deux24,
    Prime24,
    Sim,
    Replay
  }

  /**
   * @param <T> Any Type of Data
   * @param deux25 Returns if it is the Duex 2025 robot
   * @param prime25 Returns this value if it is Prime 2025 robot
   * @param deux24 Returns if it is Duex 2024
   * @param prime24 Returns if it is Prime 2024
   * @return returns duex25 parm if it is Duex 2025 robot, returns prime25 parameter if it is Prime
   *     2025 robot in the config. Defaults to duex25.
   */
  public static <T> T robotTypeValue(T deux25, T prime25, T deux24, T prime24) {
    switch (RobotConfigConstants.robotType) {
      case Deux25:
        return deux25;
      case Prime25:
        return prime25;
      case Deux24:
        return deux24;
      case Prime24:
        return prime24;
      default:
        return deux25;
    }
  }

  public static <T> T robotTypeValue(T defaultVar, RobotTypeParm<T>... parm) {
    if (parm == null) {
      return defaultVar;
    }
    for (RobotTypeParm<T> p : parm) {
      if (p.robotType == RobotConfigConstants.robotType) {
        return p.value;
      }
    }
    return defaultVar;
  }
}
