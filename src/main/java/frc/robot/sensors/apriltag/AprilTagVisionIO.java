package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {
  @AutoLog
  public static class AprilTagVisionIOInputs {
    public boolean connected = false;
    public AprilTagObservation latestTargetObservation =
        new AprilTagObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  public static record AprilTagObservation(Rotation2d tx, Rotation2d ty) {}

  public static record PoseObservation(
      double ambiguity, double timestamp, Pose3d pose, int tagCount, double averageTagDistance) {}

  public default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
