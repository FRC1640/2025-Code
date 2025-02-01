package frc.robot.util.tools;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class DistanceManager {
  /**
   * Returns the nearest distance from the positions from the checked points
   *
   * @param robotPos The current robot position
   * @param checkPoints The checking points (A position 2D array with all the positions that you
   *     need to check)
   * @return the distance from the nearest points
   */
  public static double getNearestPositionDistance(Pose2d robotPos, Pose2d[] checkPoints) {
    double distance = Double.MAX_VALUE;
    for (Pose2d pos : checkPoints) {
      double distanceLocalPos = robotPos.getTranslation().getDistance(pos.getTranslation());
      if (distance > distanceLocalPos) {
        distance = distanceLocalPos;
      }
    }
    return distance;
  }
  /**
   * Returns the nearest position from the positions from the checked points
   *
   * @param robotPos The current robot position
   * @param checkPoints The checking points (A position 2D array with all the positions that you
   *     need to check)
   * @return the nearest points
   */
  public static Pose2d getNearestPosition(Pose2d robotPos, Pose2d[] checkPoints) {
    double distance = Double.MAX_VALUE;
    Pose2d nearestPos = new Pose2d();
    for (Pose2d pos : checkPoints) {
      double distanceLocalPos = robotPos.getTranslation().getDistance(pos.getTranslation());
      if (distance > distanceLocalPos) {
        distance = distanceLocalPos;
        nearestPos = pos;
      }
    }
    return nearestPos;
  }

  public static Pose2d getNearestPosition(
      Pose2d robotPos, Pose2d[] checkPoints, Function<Pose2d, Pose2d> poseFunction) {
    double distance = Double.MAX_VALUE;
    Pose2d nearestPos = new Pose2d();
    for (Pose2d pos : checkPoints) {
      double distanceLocalPos = robotPos.getTranslation().getDistance(pos.getTranslation());
      if (distance > distanceLocalPos) {
        Logger.recordOutput("INPUT", robotPos);
        Logger.recordOutput("OUTPUT", poseFunction.apply(robotPos));
        distance = distanceLocalPos;
        nearestPos = pos;
      }
    }
    return nearestPos;
  }
}
