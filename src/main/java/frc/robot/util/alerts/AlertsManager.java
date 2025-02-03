package frc.robot.util.alerts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.dashboard.Elastic;
import frc.robot.util.dashboard.Elastic.Notification;
import frc.robot.util.dashboard.Elastic.Notification.NotificationLevel;
import frc.robot.util.periodic.PeriodicBase;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class AlertsManager extends PeriodicBase {

  private static ArrayList<Alert> alerts = new ArrayList<Alert>();
  private static ArrayList<BooleanSupplier> alertConditions = new ArrayList<BooleanSupplier>();

  public AlertsManager() {}

  /**
   * Creates a new alert. Call this method once when instantiating the device/subsystem.
   *
   * @param condition
   * @param message
   * @param type
   */
  public static void addAlert(BooleanSupplier condition, String message, AlertType type) {
    Alert alert = new Alert(message, type);
    alerts.add(alert);
    alertConditions.add(condition);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < alerts.size(); i++) {
      boolean alertGet = alertConditions.get(i).getAsBoolean();

      if (alertGet == false) {
        NotificationLevel notifLevel;

        switch (alerts.get(i).getType()) {
          case kError:
            notifLevel = NotificationLevel.ERROR;
            break;
          case kWarning:
            notifLevel = NotificationLevel.WARNING;
            break;
          case kInfo:
            notifLevel = NotificationLevel.INFO;
            break;
          default:
            notifLevel = NotificationLevel.WARNING;
        }
        Elastic.sendNotification(
            new Notification(
                notifLevel, notifLevel + " - " + alerts.get(i).getText(), alerts.get(i).getText()));
      }

      alerts.get(i).set(alertGet);
    }
  }
}
