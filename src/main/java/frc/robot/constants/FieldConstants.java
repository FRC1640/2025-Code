package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.tools.WPICal.AprilTagPositionSwitcher;
import frc.robot.util.tools.WPICal.AprilTagPositionSwitcher.AprilTagSetting;
import org.photonvision.simulation.VisionSystemSim;

public class FieldConstants {
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
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

  public static final Pose2d[] reefPositionsRed =
      new Pose2d[] {
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(
                    new Translation2d(12.227305999999999, 4.0259))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(10)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                // TODO do

                // .addValue(
                //     AprilTagSetting.WPICal
                .get(),
            Rotation2d.fromDegrees(360)),
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(new Translation2d(12.64335, 4.745482))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(9)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(300)),
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(new Translation2d(13.474446, 4.745482))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(8)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(240)),
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(new Translation2d(13.890498, 4.0259))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(7)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(
                    new Translation2d(13.474446, 3.3063179999999996))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(6)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(120)),
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(
                    new Translation2d(12.643358, 3.3063179999999996))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(11)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(60))
      };

  public static final Pose2d[] reefPositionsBlue =
      new Pose2d[] {
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(new Translation2d(5.321046, 4.0259))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(21)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(
                    new Translation2d(4.904739999999999, 3.3063179999999996))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(22)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(480)),
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(
                    new Translation2d(4.073905999999999, 3.3063179999999996))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(17)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(420)),
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(new Translation2d(3.6576, 4.0259))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(18)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(360)),
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(
                    new Translation2d(4.073905999999999, 4.745482))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(19)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(300)),
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(
                    new Translation2d(4.904739999999999, 4.745482))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(20)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(240)),
      };

  public static final Translation2d reefCenterPosRed =
      new AprilTagPositionSwitcher<Translation2d>(new Translation2d(13.07, 4)).get();

  public static final Translation2d reefCenterPosBlue =
      new AprilTagPositionSwitcher<Translation2d>(new Translation2d(4.47, 4)).get();

  public static final Pose2d processorPositionRed =
      new Pose2d(
          new AprilTagPositionSwitcher<Translation2d>(new Translation2d(6.0, 0.6)).get(),
          Rotation2d.fromDegrees(270));

  public static final Pose2d processorPositionBlue =
      new Pose2d(
          new AprilTagPositionSwitcher<Translation2d>(new Translation2d(11.55, 7.5)).get(),
          Rotation2d.fromDegrees(90));

  // Based off of the 2025-reefscape.json
  public static final Pose2d[] coralStationPosBlue =
      new Pose2d[] {
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(new Translation2d(0.851154, 0.65532))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(12)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(54)), // 12
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(
                    new Translation2d(0.851154, 7.3964799999999995))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(13)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(306)) // 13
      };
  public static final Pose2d[] coralStationPosRed =
      new Pose2d[] {
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(new Translation2d(16.697198, 0.65532))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(1)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(126)), // 1
        new Pose2d(
            new AprilTagPositionSwitcher<Translation2d>(
                    new Translation2d(16.697198, 7.3964799999999995))
                .addValue(
                    AprilTagSetting.WPILib,
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
                        .getTagPose(2)
                        .get()
                        .getTranslation()
                        .toTranslation2d())
                .get(),
            Rotation2d.fromDegrees(234)) // 2
      };
}
