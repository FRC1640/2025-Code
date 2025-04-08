package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.WPICal.AprilTagPositionSwitcher;
import frc.robot.util.WPICal.AprilTagPositionSwitcher.AprilTagSetting;
import frc.robot.util.WPICal.WPICALPosManager;
import org.photonvision.simulation.VisionSystemSim;

public class FieldConstants {
  public static final AprilTagFieldLayout aprilTagLayout =
      new AprilTagPositionSwitcher<AprilTagFieldLayout>(
              AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded))
          .addValue(
              AprilTagSetting.WPILibAndyMark,
              AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark))
          .addValue(AprilTagSetting.WPICal, WPICALPosManager.aprilTagFieldLayout)
          .get();
  public static final int tagCount = 22;

  public static final VisionSystemSim visionSim = new VisionSystemSim("main");

  public static final double height = 8.21;
  public static final double width = 16.54;

  public static final int kNorth = 0;
  public static final int kNorthEast = 1;
  public static final int kSouthEast = 2;
  public static final int kSouth = 3;
  public static final int kSouthWest = 4;
  public static final int kNorthWest = 5;

  public static final double reefPoleRegularHeight = 0.568;
  public static final Pose2d[] reefPositionsRed =
      new Pose2d[] {
        new Pose2d(13.890498, 4.0259, Rotation2d.fromDegrees(180)), // id 7
        new Pose2d(13.474446, 3.3063179999999996, Rotation2d.fromDegrees(120)), // id 6
        new Pose2d(12.643358, 3.3063179999999996, Rotation2d.fromDegrees(60)), // id 11
        new Pose2d(12.227305999999999, 4.0259, Rotation2d.fromDegrees(360)), // id 10
        new Pose2d(12.643358, 4.745482, Rotation2d.fromDegrees(300)), // id 9
        new Pose2d(13.474446, 4.745482, Rotation2d.fromDegrees(240)), // id 8
      };

  public static final Pose2d[] reefPositionsBlue =
      new Pose2d[] {
        new Pose2d(3.6576, 4.0259, Rotation2d.fromDegrees(360)), // id 18
        new Pose2d(4.073905999999999, 4.745482, Rotation2d.fromDegrees(300)), // id 19
        new Pose2d(4.904739999999999, 4.745482, Rotation2d.fromDegrees(240)), // id 20
        new Pose2d(5.321046, 4.0259, Rotation2d.fromDegrees(180)), // id 21
        new Pose2d(4.904739999999999, 3.3063179999999996, Rotation2d.fromDegrees(480)), // id 22
        new Pose2d(4.073905999999999, 3.3063179999999996, Rotation2d.fromDegrees(420)), // id 17
      };

  public static final Translation2d reefCenterPosRed = new Translation2d(13.07, 4);

  public static final Translation2d reefCenterPosBlue = new Translation2d(4.47, 4);

  public static final Pose2d processorPositionRed =
      new Pose2d(11.55, 7.5, Rotation2d.fromDegrees(90));

  public static final Pose2d processorPositionBlue =
      new Pose2d(6.0, 0.6, Rotation2d.fromDegrees(270));

  // Based off of the 2025-reefscape.json
  public static final Pose2d[] coralStationPosBlue =
      new Pose2d[] {
        new Pose2d(0.851154, 0.65532, Rotation2d.fromDegrees(54)), // 12
        new Pose2d(0.851154, 7.3964799999999995, Rotation2d.fromDegrees(306)) // 13
      };
  public static final Pose2d[] coralStationPosRed =
      new Pose2d[] {
        new Pose2d(16.697198, 0.65532, Rotation2d.fromDegrees(126)), // 1
        new Pose2d(16.697198, 7.3964799999999995, Rotation2d.fromDegrees(234)) // 2
      };
  // left is +1 id
  public static final double[] reefPoleHeightBlue = {
    0.556, 0.534, 0.558, 0.563, 0.512, 0.568, 0.568, 0.568, 0.568, 0.568, 0.568, 0.568,
  };
  public static final double[] reefPoleHeightRed = {
    0.534, 0.569, 0.694, 0.531, 0.595, 0.548, 0.568, 0.568, 0.568, 0.568, 0.568, 0.568,
  };
}
