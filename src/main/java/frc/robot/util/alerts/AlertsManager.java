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
  public static ArrayList<Notification> notificationQueue = new ArrayList<Notification>();
  private static ArrayList<String> ranNotifications = new ArrayList<String>();
  private int notificationQueueIndex = 0;
  private Long currentNotificationDelay = Long.valueOf(0);

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

  private long lastNotificationTime = 0;

  @Override
  public void periodic() {

    for (int i = 0; i < alerts.size(); i++) {
      boolean alertGet = alertConditions.get(i).getAsBoolean();

      if (alertGet == true) {
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
        Notification notif =
            new Notification(
                notifLevel, notifLevel + " - " + alerts.get(i).getText(), alerts.get(i).getText());
        if (!ranNotifications.contains(notif.getTitle())) {
          notificationQueue.add(notif);
          ranNotifications.add(notif.getTitle());
        } else {
        }
      }

      alerts.get(i).set(alertGet);
    }
    // sending notifications
    if (!notificationQueue.isEmpty()
        && notificationQueueIndex < notificationQueue.size()
        && notificationQueue.get(notificationQueueIndex) != null
        && activeNotification(currentNotificationDelay)) {

      Elastic.sendNotification(notificationQueue.get(notificationQueueIndex));
      currentNotificationDelay =
          Long.valueOf(notificationQueue.get(notificationQueueIndex).getDisplayTimeMillis());
      notificationQueueIndex++;
    }
  }

  public Boolean activeNotification(Long notificationDelay) {
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastNotificationTime + 0.01 >= notificationDelay) {
      lastNotificationTime = currentTime;
      return true;
    }
    return false;
  }
}
