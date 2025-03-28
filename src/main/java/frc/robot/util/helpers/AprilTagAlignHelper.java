package frc.robot.util.helpers;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.apriltag.AprilTagVision.FiducialVector;
import java.util.ArrayList;
import java.util.Optional;

public class AprilTagAlignHelper {
  private AprilTagVision[] visions = {};
  private static AprilTagAlignHelper instance = new AprilTagAlignHelper();

  public AprilTagAlignHelper() {}

  public AprilTagAlignHelper(AprilTagVision... visions) {
    this.visions = visions;
    AprilTagAlignHelper.instance = this;
  }

  public static Optional<Translation2d> getAverageLocalAlignVector(int id) {
    ArrayList<FiducialVector[]> allVectors = getLocalAlignVectors();
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
    return Optional.of(new Translation2d(ki / total, kj / total));
  }

  public static ArrayList<FiducialVector[]> getLocalAlignVectors() {
    ArrayList<FiducialVector[]> vectors = new ArrayList<>();
    for (AprilTagVision vision : instance.visions) {
      vectors.add(vision.getLocalAlignVectors());
    }
    return vectors;
  }
}
