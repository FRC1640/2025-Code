package frc.robot.util.tools;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.FieldConstants;

public class DistanceManager {
  /**
   * Returns the nearest distance from the positions from the checked points
   *
   * @param robotPos The current robot position
   * @param checkPoints The checking points (A position 2D array with all the positions that you
   *     need to check)
   * @return the distance from the nearest points
   */
  public static double getNearestPosition(Pose2d robotPos, Pose2d[] checkPoints) {
    double distance = Double.MAX_VALUE;
    for (Pose2d pos : FieldConstants.coralStationPosRed) {
      double distanceLocalPos = robotPos.getTranslation().getDistance(pos.getTranslation());
      if (distance > distanceLocalPos) {
        distance = distanceLocalPos;
      }
    }
    return distance;
  }
}
