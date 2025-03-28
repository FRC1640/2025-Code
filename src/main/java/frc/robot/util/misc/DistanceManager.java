package frc.robot.util.misc;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;

import java.util.ArrayList;
import java.util.function.Function;
import java.util.stream.IntStream;

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
    for (Pose2d pos1 : checkPoints) {
      Pose2d pos = poseFunction.apply(pos1);
      double distanceLocalPos = robotPos.getTranslation().getDistance(pos.getTranslation());
      if (distance > distanceLocalPos) {
        distance = distanceLocalPos;
        nearestPos = pos;
      }
    }
    return nearestPos;
  }

  public static double getPositionDistance(Pose2d robotPos, Pose2d targetPos) {
    return robotPos.getTranslation().getDistance(targetPos.getTranslation());
  }

  public static Pose2d addRotatedDim(Pose2d pose2d, double dim, Rotation2d rot) {
    Translation2d translation =
        pose2d.getTranslation().minus(new Translation2d(dim, 0).rotateBy(rot));

    Logger.recordOutput("OUTPUT", new Pose2d(translation, pose2d.getRotation()));
    return new Pose2d(translation, pose2d.getRotation());
  }

  public static AprilTag getAutoalignTagId(Pose2d target) {
    ArrayList<AprilTag> autoalignTags = new ArrayList<>();
    IntStream.of(
            AllianceManager.chooseFromAlliance(
                new int[] {17, 18, 19, 20, 21, 22}, new int[] {6, 7, 8, 9, 10, 11}))
        .forEach(
            (i) ->
                autoalignTags.add(
                    new AprilTag(i, FieldConstants.aprilTagLayout.getTagPose(i).get())));
    AprilTag nearestTag = autoalignTags.get(0);
    double nearestDist = Double.MAX_VALUE;
    for (AprilTag tag : autoalignTags) {
      double dist =
          target.getTranslation().getDistance(tag.pose.getTranslation().toTranslation2d());
      if (dist < nearestDist) {
        nearestTag = tag;
        nearestDist = dist;
      }
    }
    return nearestTag;
  }
}
