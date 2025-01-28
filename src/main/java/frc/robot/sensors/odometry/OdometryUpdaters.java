package frc.robot.sensors.odometry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.Robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.apriltag.AprilTagVisionIO.PoseObservation;
import java.util.LinkedList;
import java.util.List;

public class OdometryUpdaters {
  private static boolean USE_APRILTAGS_IN_AUTO = false;

  public static void standardUpdater(SwerveDrivePoseEstimator estimator, AprilTagVision[] cameras) {
    for (AprilTagVision camera : cameras) {
      addVisionEstimate(estimator, camera);
    }
  }

  // #region Private Methods
  private static boolean isPoseValid(Pose2d pose) {
    return FieldConstants.width >= pose.getX()
        && FieldConstants.height >= pose.getY()
        && pose.getX() > 0
        && pose.getY() > 0;
  }

  private static void addVisionEstimate(SwerveDrivePoseEstimator estimator, AprilTagVision vision) {
    // TODO clean this up. Maybe move some of this into Vision subsystem
    if (Robot.getState() == RobotState.DISABLED) {
      return;
    }
    if (Robot.getState() == RobotState.AUTONOMOUS && !USE_APRILTAGS_IN_AUTO) {
      return;
    }

    List<Pose2d> robotPoses = new LinkedList<>();
    List<Pose2d> robotPosesAccepted = new LinkedList<>();
    List<Pose2d> robotPosesRejected = new LinkedList<>();
    for (PoseObservation poseObservation : vision.getPoses()) {
      Pose2d visionUpdate = poseObservation.pose().toPose2d();
      robotPoses.add(visionUpdate);

      if (!(isPoseValid(visionUpdate)
          && vision.isConnected()
          && poseObservation.tagCount() > 0
          && poseObservation.ambiguity() < 0.2
          && Math.abs(poseObservation.pose().getZ()) < 0.75)) {
        robotPosesRejected.add(visionUpdate);
        continue;
      }
      robotPosesAccepted.add(visionUpdate);
      double distFactor =
          Math.pow(poseObservation.averageTagDistance(), 2.0)
              / poseObservation.tagCount()
              * vision.getStandardDeviation();
      double xy = 0.1 * distFactor;
      double rot = Double.MAX_VALUE;
      if (poseObservation.ambiguity() < 0.05 && poseObservation.tagCount() > 1) {
        rot = 0.06 * distFactor;
      }

      estimator.addVisionMeasurement(
          visionUpdate, poseObservation.timestamp(), VecBuilder.fill(xy, xy, rot));
    }
  }
  // #endregion
}
