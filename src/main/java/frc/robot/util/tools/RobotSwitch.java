package frc.robot.util.tools;

import frc.robot.constants.RobotConstants.RobotConfigConstants;

public class RobotSwitch {

  public enum RobotType {
    Duex25,
    Prime25,
  }
  /**
   * @param <T> Any Type of Data
   * @param duex25 Returns if it is the Duex 2025 robot
   * @param prime25 Returns this value if it is Prime 2025 robot
   * @return returns duex25 parm if it is Duex 2025 robot, returns prime25 parameter if it is Prime
   *     2025 robot in the config
   */
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
