package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
  public static final int kNorth = 0;
  public static final int kNorthEast = 1;
  public static final int kSouthEast = 2;
  public static final int kSouth = 3;
  public static final int kSouthWest = 4;
  public static final int kNorthWest = 5;
  public static final Pose2d[] reefPositionsRed = new Pose2d[] {
    new Pose2d(12.227305999999999, 4.0259, Rotation2d.fromDegrees(180)),
    new Pose2d(12.643358, 4.745482, Rotation2d.fromDegrees(120)),
    new Pose2d(13.474446, 4.745482, Rotation2d.fromDegrees(60)),
    new Pose2d(13.890498, 4.0259, Rotation2d.fromDegrees(0)),
    new Pose2d(13.474446, 3.3063179999999996, Rotation2d.fromDegrees(300)),
    new Pose2d(12.643358, 4.745482, Rotation2d.fromDegrees(240)),
  };

  public static final Pose2d[] reefPositionsBlue = new Pose2d[] {
    new Pose2d(5.321046, 4.0259, Rotation2d.fromDegrees(0)),
    new Pose2d(4.904739999999999, 3.3063179999999996, Rotation2d.fromDegrees(300)),
    new Pose2d(4.073905999999999, 4.745482, Rotation2d.fromDegrees(240)),
    new Pose2d(3.6576, 4.0259, Rotation2d.fromDegrees(180)),
    new Pose2d(4.073905999999999, 4.745482, Rotation2d.fromDegrees(120)),
    new Pose2d(4.904739999999999, 3.3063179999999996, Rotation2d.fromDegrees(60)),
  };
}
