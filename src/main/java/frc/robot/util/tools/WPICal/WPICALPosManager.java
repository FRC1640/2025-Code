package frc.robot.util.tools.WPICal;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.RobotConstants.AprilTagPositionSettings;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.tools.WPICal.AprilTagPositionSwitcher.AprilTagSetting;
import java.io.IOException;

public class WPICALPosManager {
  public static AprilTagFieldLayout aprilTagFieldLayout;

  static {
    if (AprilTagPositionSettings.fieldPositionType == AprilTagSetting.WPICal) {
      AlertsManager.addAlert(
          () -> aprilTagFieldLayout == null,
          "Error - WPICal JSON file does not exist",
          AlertType.kError);
      try {
        aprilTagFieldLayout =
            AprilTagFieldLayout.loadFromResource(AprilTagPositionSettings.WPICalOutputJson);
      } catch (IOException e) {
        e.printStackTrace();
        aprilTagFieldLayout = null;
      }
      aprilTagFieldLayout = null;
    }
  }

  public static Translation2d getPositionID(int id) {
    if (aprilTagFieldLayout != null) {
      return aprilTagFieldLayout.getTagPose(id).get().getTranslation().toTranslation2d();
    }
    return null;
  }
}
