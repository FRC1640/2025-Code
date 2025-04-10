package frc.robot.util.helpers;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.util.misc.AllianceManager;
import java.util.ArrayList;
import java.util.Optional;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

public class AprilTagAlignHelper {
  private AprilTagAlignHelper() {}

  public static Optional<Translation2d> getAverageLocalAlignVector(
      int id, AprilTagVision... visions) {
    Translation2d[] vectors = getLocalAlignVectors(id, visions).toArray(Translation2d[]::new);
    Translation2d average = null;
    if (vectors.length != 0) {
      double ki = 0, kj = 0;
      int total = 0;
      for (Translation2d vector : vectors) {
        ki += vector.getX();
        kj += vector.getY();
        total++;
      }
      average = new Translation2d(ki / total, kj / total);
    }
    if (average != null) {
      Logger.recordOutput("LocalTagAlign/averageVector", average);
    }
    return Optional.ofNullable(average);
  }

  private static ArrayList<Translation2d> getLocalAlignVectors(int id, AprilTagVision[] visions) {
    ArrayList<Translation2d> vectors = new ArrayList<>();
    for (AprilTagVision vision : visions) {
      vision.setIDToUse(id);
      Optional<Translation2d> vector = vision.getLocalAlignVector();
      if (vector.isPresent()) {
        vectors.add(vector.get());
      }
    }
    return vectors;
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

  @Deprecated
  public static Optional<Translation2d> getPoleAlignVector(
      int id, CoralPreset preset, AprilTagVision... visions) {
    Optional<Translation2d> averageVector = getAverageLocalAlignVector(id, visions);
    if (averageVector.isPresent()) {
      averageVector =
          Optional.of(
              averageVector
                  .get()
                  .plus(new Translation2d(0, preset.getGantrySetpoint(true).alignOffset)));
    }
    return averageVector;
  }
}
