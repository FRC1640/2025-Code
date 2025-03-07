package frc.robot.util.tools.WPICal;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.RobotConstants.AprilTagPositionSettings;
import frc.robot.util.tools.WPICal.AprilTagPositionSwitcher.AprilTagSetting;
import java.io.IOException;
import java.nio.file.Path;

public class WPICALPosManager {
  public static AprilTagFieldLayout aprilTagFieldLayout;

  static {
    if (AprilTagPositionSettings.fieldPositionType == AprilTagSetting.WPICal) {
      try {
        aprilTagFieldLayout =
            new AprilTagFieldLayout(
                Path.of(
                    Filesystem.getDeployDirectory()
                        + "/resources/"
                        + AprilTagPositionSettings.WPICalOutputJson));
      } catch (IOException e) {
        e.printStackTrace();
        aprilTagFieldLayout = null;
      }
    }
  }

  public static Translation2d getPositionID(int id) {
    if (aprilTagFieldLayout != null) {
      return aprilTagFieldLayout.getTagPose(id).get().getTranslation().toTranslation2d();
    }
    return null;
  }
}
