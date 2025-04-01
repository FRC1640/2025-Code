package frc.robot.util.helpers;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.apriltag.AprilTagVision.FiducialVector;
import frc.robot.util.misc.AllianceManager;
import java.util.ArrayList;
import java.util.Optional;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

public class AprilTagAlignHelper {
  private AprilTagAlignHelper() {}

  public static Optional<Translation2d> getOffsetAlignVector(int id, CoralPreset preset) {
    Optional<Translation2d> averageVector = getAverageLocalAlignVector(id);
    if (averageVector.isPresent()) {
      averageVector =
          Optional.of(
              averageVector
                  .get()
                  .plus(new Translation2d(0, preset.getGantrySetpoint(true).alignOffset)));
    }
    return averageVector;
  }

  public static Optional<Translation2d> getAverageLocalAlignVector(
      int id, AprilTagVision... visions) {
    ArrayList<FiducialVector[]> allVectors = getLocalAlignVectors(visions);
    ArrayList<FiducialVector> filteredVectors = new ArrayList<>();
    for (FiducialVector[] visionVectors : allVectors) {
      Optional<FiducialVector> vector = Optional.empty();
      for (FiducialVector tagVector : visionVectors) {
        if (tagVector.id() == id) {
          if (vector.isPresent() && vector.get().timestamp() < tagVector.timestamp()) {
            vector = Optional.of(tagVector);
          } else if (vector.isEmpty()) {
            vector = Optional.of(tagVector);
          }
        }
      }
      if (vector.isPresent()) {
        filteredVectors.add(vector.get());
      }
    }
    if (filteredVectors.isEmpty()) {
      return Optional.empty();
    }
    double ki = 0, kj = 0;
    int total = 0;
    for (FiducialVector vector : filteredVectors) {
      ki += vector.vector().getX();
      kj += vector.vector().getY();
      total++;
    }
    Translation2d average = new Translation2d(ki / total, kj / total);
    Logger.recordOutput("A_DEBUG/avgLocalVector", average);
    return Optional.of(average);
  }

  private static ArrayList<FiducialVector[]> getLocalAlignVectors(AprilTagVision[] visions) {
    ArrayList<FiducialVector[]> vectors = new ArrayList<>();
    for (AprilTagVision vision : visions) {
      vectors.add(vision.getLocalAlignVectors());
    }
    Logger.recordOutput("A_DEBUG/goodbye", !vectors.isEmpty());
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
}
