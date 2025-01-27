package frc.robot.util.alerts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class AlertsManager {

  public static ArrayList<Alert> alerts;
  public static ArrayList<BooleanSupplier> alertConditions;

  public static void addAlert(BooleanSupplier condition, String message, AlertType type) {
    Alert alert = new Alert(message, type);
    alerts.add(alert);
    alertConditions.add(condition);
    new Trigger(condition)
        .onTrue(new InstantCommand(() -> alert.set(true)))
        .onFalse(new InstantCommand(() -> alert.set(false)));
  }
}
