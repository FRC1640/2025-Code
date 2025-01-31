package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {
  @AutoLog
  public static class AprilTagVisionIOInputs {
    public boolean connected = false;
    public TrigTargetObservation[] trigTargetObservations = new TrigTargetObservation[0];
    public PoseObservation[] photonPoseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public Transform3d cameraDisplacement;
  }

  public static record TrigTargetObservation(
      Rotation2d tx,
      Rotation2d ty,
      double distance,
      Pose3d targetTransform,
      double timestamp,
      int fiducialId,
      double distance2D,
      Transform3d camToTarget) {}

  public static record PoseObservation(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

  public default void updateInputs(AprilTagVisionIOInputs inputs) {}

  public default Transform3d getCameraDisplacement() {
    return null;
  }
}
