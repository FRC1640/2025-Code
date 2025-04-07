package frc.robot.util.dashboard.notification;

import edu.wpi.first.wpilibj.Alert;
import frc.robot.util.dashboard.Elastic;
import frc.robot.util.dashboard.Elastic.Notification;
import frc.robot.util.dashboard.Elastic.Notification.NotificationLevel;
import java.util.ArrayList;

public class NotificationManager {
  public static ArrayList<Notification> notificationQueue = new ArrayList<Notification>();
  private static ArrayList<String> ranNotifications = new ArrayList<String>();

  private static int notificationQueueIndex = 0;
  private static Long currentNotificationDelay = Long.valueOf(0);
  private static long lastNotificationTime = 0;

  public static void alertNotification(boolean alertGet, Alert alerts) {

    if (alertGet == true) {
      NotificationLevel notifLevel;

      switch (alerts.getType()) {
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
          new Notification(notifLevel, notifLevel + " - " + alerts.getText(), alerts.getText());
      if (!ranNotifications.contains(notif.getTitle())) {
        notificationQueue.add(notif);
        ranNotifications.add(notif.getTitle());
      } else {
        notificationQueueRun();
      }
    }
  }

  public static void notificationQueueRun() {
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

  public static Boolean activeNotification(Long notificationDelay) {
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastNotificationTime + 0.01 >= notificationDelay) {
      lastNotificationTime = currentTime;
      return true;
    }
    return false;
  }

  public static void addNotification(Notification notification, Long notificationLength) {}
}
