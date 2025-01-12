package frc.robot.util.alerts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.periodic.PeriodicBase;

public class RobotAlerts extends PeriodicBase {
  private Alert lowBatteryAlert;

  public RobotAlerts() {
    constructAlerts();
  }

  // Initialize alerts
  public void constructAlerts() {
    lowBatteryAlert = new Alert("Low Battery", AlertType.kWarning);
  }

  @Override
  public void periodic() {
    double currentVoltage = checkBatteryVoltage();
    if (currentVoltage < 10.5) {
      lowBatteryAlert.set(true); // Show the alert
    }
  }

  // Check battery voltage
  public double checkBatteryVoltage() {
    double voltage = RobotController.getBatteryVoltage();
    System.out.println("Current Battery Voltage: " + voltage + "V");
    return voltage;
  }
  }
}
